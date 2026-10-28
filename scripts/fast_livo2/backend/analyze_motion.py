#!/usr/bin/env python3
"""Summarize a frozen real-data graph; render honest before/after comparisons."""
import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial import cKDTree

from validate_motion import matrix,read_pcd


def world(points,pose):
    T=matrix(pose)
    return points[:,:3]@T[:3,:3].T+T[:3,3]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--video',action='store_true')
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    snapshot=args.run/'snapshot';m=json.loads((snapshot/'manifest.json').read_text())
    result=json.loads((args.run/'result.json').read_text())
    if not result['passed']:raise ValueError('Cannot analyze a failed run as accepted')
    rows=[json.loads(s) for s in (snapshot/'candidates.jsonl').read_text().splitlines()]
    accepted=[r for r in rows if r['decision']=='accepted_and_optimized']
    frames=m['keyframes'];by_scan={f['scan_id']:f for f in frames}
    metrics=[]
    for row in accepted:
        source=frames[row['j']]
        cloud=read_pcd(snapshot/source['directory']/'match.pcd')
        target_frames=[by_scan[i] for i in row['target_scan_ids']]
        clouds=[read_pcd(snapshot/f['directory']/'match.pcd') for f in target_frames]
        before=np.concatenate([world(c,f['U_local_imu']) for c,f in zip(clouds,target_frames)])
        after=np.concatenate([world(c,f['X_map_imu']) for c,f in zip(clouds,target_frames)])
        raw_dist=cKDTree(before).query(world(cloud,source['U_local_imu']),workers=1)[0]
        opt_dist=cKDTree(after).query(world(cloud,source['X_map_imu']),workers=1)[0]
        mask=raw_dist<.5  # Same source points before/after; do not select on improved values.
        metrics.append({'i':row['i'],'j':row['j'],'source_points':len(cloud),
            'fixed_source_points':int(mask.sum()),'raw_nn_rmse_m':float(np.sqrt(np.mean(raw_dist[mask]**2))),
            'optimized_nn_rmse_m':float(np.sqrt(np.mean(opt_dist[mask]**2))),
            'raw_overlap_0_5m':float(np.mean(raw_dist<.5)),'optimized_overlap_0_5m':float(np.mean(opt_dist<.5)),
            'graph_error_before':row['graph_error_before_same_factors'],
            'graph_error_after':row['graph_error_after_same_factors']})
    if metrics:
        with (out/'loop_geometry.csv').open('w') as handle:
            writer=csv.DictWriter(handle,fieldnames=metrics[0]);writer.writeheader();writer.writerows(metrics)
    U=np.array([matrix(f['U_local_imu'])[:3,3] for f in frames])
    X=np.array([matrix(f['X_map_imu'])[:3,3] for f in frames])
    corrections=np.linalg.norm(X-U,axis=1)
    summary={'scope':'same frozen graph, same source scan IDs; consistency metrics, not ground-truth accuracy',
        'session_id':m['session_id'],'graph_version':m['graph_version'],'keyframes':len(frames),
        'accepted_loops':len(accepted),'loop_geometry':metrics,
        'mean_loop_nn_rmse_raw_m':float(np.mean([r['raw_nn_rmse_m'] for r in metrics])) if metrics else None,
        'mean_loop_nn_rmse_optimized_m':float(np.mean([r['optimized_nn_rmse_m'] for r in metrics])) if metrics else None,
        'max_keyframe_translation_correction_m':float(corrections.max()),
        'physical_sync_proven':False,'absolute_accuracy_validated':False}
    (out/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    axes[0,0].plot(U[:,0],U[:,1],label='Raw frontend',color='#d49136')
    axes[0,0].plot(X[:,0],X[:,1],label='Optimized',color='#149e8d')
    axes[0,0].axis('equal');axes[0,0].legend();axes[0,0].set(title='Same keyframe timestamps',xlabel='Local x (m)',ylabel='Local y (m)')
    axes[0,1].plot(corrections*100,color='#149e8d');axes[0,1].set(title='Translation correction',xlabel='Keyframe ID',ylabel='Correction (cm)')
    for label,key,color in [('Before','graph_error_before','#d49136'),('After','graph_error_after','#149e8d')]:
        axes[1,0].plot([r[key] for r in metrics],marker='.',label=label,color=color)
    axes[1,0].set(title='Same-factor objective at each loop commit',xlabel='Accepted loop update',ylabel='Robust graph error');axes[1,0].legend()
    for label,key,color in [('Raw','raw_nn_rmse_m','#d49136'),('Optimized','optimized_nn_rmse_m','#149e8d')]:
        axes[1,1].plot([r[key]*100 for r in metrics],marker='.',label=label,color=color)
    axes[1,1].set(title='Historical submap NN consistency',xlabel='Accepted loop',ylabel='NN RMSE (cm)');axes[1,1].legend()
    fig.suptitle('FAST-LIVO2 Stage B | recorded motion | consistency, not absolute accuracy')
    fig.tight_layout();fig.savefig(out/'comparison.png',dpi=160);plt.close(fig)
    if args.video:render_video(snapshot,frames,U,X,result,summary,out)
    print(json.dumps(summary,indent=2),flush=True)


def render_video(snapshot,frames,U,X,result,analysis,out):
    cv2.setNumThreads(1)
    local=[read_pcd(snapshot/f['directory']/'full.pcd') for f in frames]
    raw=np.concatenate([world(c,f['U_local_imu']) for c,f in zip(local,frames)])
    optimized=np.concatenate([world(c,f['X_map_imu']) for c,f in zip(local,frames)])
    lo=U.min(axis=0)-[5,5,2.5];hi=U.max(axis=0)+[5,5,2.5];center=(lo+hi)/2
    mask=np.all((raw>=lo)&(raw<=hi),axis=1)
    raw=raw[mask];optimized=optimized[mask]
    # Use identical source indices for the two clouds; only their poses differ.
    _,selection=np.unique(np.floor(raw/.08).astype(np.int32),axis=0,return_index=True)
    raw,optimized=raw[selection],optimized[selection]
    zlo,zhi=np.percentile(raw[:,2],[2,98]);palette=cv2.applyColorMap((np.clip((raw[:,2]-zlo)/max(.1,zhi-zlo),0,1)*255).astype('uint8')[:,None],cv2.COLORMAP_TURBO).reshape(-1,3)
    image=Image.new('RGB',(1920,1080),(13,20,32));draw=ImageDraw.Draw(image)
    font=lambda n:ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',n)
    draw.text((34,20),'FAST-LIVO2  |  独立回环后端',font=font(39),fill=(235,243,255))
    draw.text((35,78),'真实运动数据 · 同一最终图版本 · 优化前后历史地图重建',font=font(22),fill=(142,175,199))
    for left,title in [(30,'原始前端关键帧地图'),(975,'iSAM2 优化关键帧地图')]:
        draw.rounded_rectangle((left,130,left+915,820),radius=14,fill=(19,30,46),outline=(45,62,83),width=2)
        draw.text((left+23,146),title,font=font(26),fill=(220,234,244))
    draw.text((35,854),f'{len(frames)} 个关键帧   |   {analysis["accepted_loops"]} 个接受并优化的回环',font=font(28),fill=(123,226,189))
    draw.text((35,909),f'最大关键帧平移修正：{analysis["max_keyframe_translation_correction_m"]*100:.1f} cm',font=font(25),fill=(210,226,241))
    draw.text((35,956),f'首尾距离：{result["raw_endpoint_distance_m"]*100:.1f} cm → {result["optimized_endpoint_distance_m"]*100:.1f} cm',font=font(25),fill=(210,226,241))
    draw.text((1010,861),'局部视图：轨迹范围外扩5m',font=font(24),fill=(178,200,220))
    draw.text((1010,911),'原始扫描保留；点云仅按各自位姿变换一次',font=font(23),fill=(178,200,220))
    draw.text((1010,960),'闭合与几何一致性指标不等于绝对定位精度',font=font(23),fill=(178,200,220))
    draw.text((35,1034),'OFFLINE VALIDATION  /  PCL ICP + GTSAM iSAM2  /  NO GROUND TRUTH',font=font(18),fill=(132,157,184))
    base=np.asarray(image)[:,:,::-1].copy()
    writer=cv2.VideoWriter(str(out/'comparison_1080p.mp4'),cv2.VideoWriter_fourcc(*'avc1'),20,(1920,1080))
    if not writer.isOpened():raise RuntimeError('H264 encoder unavailable')
    def project(points,yaw,left):
        elev=.65;right=np.array([math.cos(yaw),-math.sin(yaw),0])
        up=np.array([math.sin(yaw)*math.sin(elev),math.cos(yaw)*math.sin(elev),math.cos(elev)])
        R=np.column_stack([right,up,np.cross(right,up)])
        p=(points-center)@R
        return np.column_stack([left+458+p[:,0]*43,510-p[:,1]*43]).astype(np.int32),p[:,2]
    try:
        for i in range(480):
            yaw=.5+.65*i/479;canvas=base.copy()
            for points,path,left,color in [(raw,U,30,(64,183,230)),(optimized,X,975,(152,226,112))]:
                uv,depth=project(points,yaw,left)
                valid=(uv[:,0]>left+12)&(uv[:,0]<left+903)&(uv[:,1]>194)&(uv[:,1]<806)
                order=np.argsort(depth[valid]);q=uv[valid][order];c=palette[valid][order]
                for dx,dy in [(0,0),(1,0),(0,1)]:canvas[q[:,1]+dy,q[:,0]+dx]=c
                p,_=project(path,yaw,left);cv2.polylines(canvas,[p.reshape(-1,1,2)],False,color,3,cv2.LINE_AA)
                cv2.circle(canvas,tuple(p[0]),6,(255,255,255),-1,cv2.LINE_AA)
            writer.write(canvas)
            if i==240:cv2.imwrite(str(out/'comparison_cover.jpg'),canvas)
    finally:writer.release()
    capture=cv2.VideoCapture(str(out/'comparison_1080p.mp4'));decoded=0
    while capture.read()[0]:decoded+=1
    capture.release()
    if decoded!=480:raise RuntimeError('Comparison video decode incomplete')
    (out/'video_checks.json').write_text(json.dumps({'frames':480,'decoded':decoded,'fps':20,'duration_s':24,'size':[1920,1080]},indent=2)+'\n')


if __name__=='__main__':main()
