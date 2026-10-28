#!/usr/bin/python3
"""Explicit solve entrypoint; each invocation creates fresh results. Never edits FAST-LIVO2."""
import argparse, datetime, json, re, shutil, subprocess, sys, os
from pathlib import Path
import numpy as np, yaml, cv2
from common import *
CHECKS=['rigid_static_confirmed','marker_ids_layout_orientation_verified','four_hole_correspondence_verified','pointcloud_support_accepted']

def run(exe,config,log):
    env=os.environ.copy();env['ROS_LOG_DIR']=str(ROOT/'logs/ros')
    with output(log).open('w') as f:
        p=subprocess.run([str(ROOT/'ws/install_integrated/fast_calib/lib/fast_calib'/exe),'--ros-args','--params-file',str(config)],stdout=f,stderr=subprocess.STDOUT,timeout=240,env=env)
    if p.returncode:raise ValueError(exe+' failed, inspect '+str(log))
def centers(path):
    text=Path(path).read_text();L=[];C=[]
    for name,target in [('lidar_centers:',L),('qr_centers:',C)]:
        for line in text.splitlines():
            if line.startswith(name):target.extend([[float(v) for v in xyz.split(',')] for xyz in re.findall(r'\{([^}]+)\}',line)])
    L=np.array(L);C=np.array(C)
    if L.shape!=(4,3) or C.shape!=(4,3) or not np.isfinite(L).all() or not np.isfinite(C).all():raise ValueError('Expected exactly ONE fresh block of four center pairs')
    return L,C

def quality(txt,blocks,params,out,feature_reviewed):
    d=read(txt);T=np.eye(4);T[:3,:3]=np.array(d['Rcl']).reshape(3,3);T[:3,3]=d['Pcl'];result=matrix_check(T)
    result.update(optimization_converged=True,feature_matching_verified=feature_reviewed,geometric_calibration_accepted=False,uncertainty=None,uncertainty_note='Upstream SVD provides no calibrated covariance; no uncertainty invented',scenes=[])
    residuals=[]
    for name,L,C,p in blocks:
        pred=(T[:3,:3]@L.T).T+T[:3,3];res=np.linalg.norm(pred-C,axis=1);residuals.extend(res.tolist())
        K=np.array([[p['fx'],0,p['cx']],[0,p['fy'],p['cy']],[0,0,1]],float);D=np.array([p[k] for k in ['k1','k2','p1','p2','k3']])
        projected=cv2.projectPoints(pred,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2);target=cv2.projectPoints(C,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
        px=np.linalg.norm(projected-target,axis=1)
        result['scenes'].append({'name':name,'lidar_centers':L.tolist(),'camera_centers':C.tolist(),'residual_m_per_hole':res.tolist(),'rmse_m':float(np.sqrt(np.mean(res**2))),'derived_hole_reprojection_error_px':px.tolist(),'derived_hole_reprojection_rmse_px':float(np.sqrt(np.mean(px**2))),'note':'Camera hole centers inferred from ArUco board pose; these are not independent observed hole pixels'})
    result['joint_rmse_m']=float(np.sqrt(np.mean(np.square(residuals))))
    output(out).write_text(yaml.safe_dump(result,sort_keys=False));return result

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['single','multi','diagnose']);p.add_argument('manifest');p.add_argument('--scene');a=p.parse_args()
    manifest=read(a.manifest);scenes=manifest['scenes']
    if a.mode in ('single','diagnose'):
        scenes=[s for s in scenes if s['name']==a.scene]
        if len(scenes)!=1:raise ValueError('Use --scene with one configured scene')
    if a.mode=='multi' and len(scenes)<3:raise ValueError('Need >=3 distinct scenes')
    if len({s['name'] for s in scenes})!=len(scenes):raise ValueError('Duplicate scenes')
    configs=[];bag_paths=set()
    for s in scenes:
        cfg=Path(s['config']).resolve();d=read(cfg);par=d['fast_calib']['ros__parameters'];prov=read(str(cfg)+'.provenance.json')
        if sha(cfg)!=prov['config_sha256']:raise ValueError('Config changed: regenerate with make-config')
        if prov['scene']!=s['name'] or prov['bag']!=par['bag_path']:raise ValueError('Config provenance mismatch')
        if sha(prov['intrinsics']['source'])!=prov['intrinsics']['sha256'] or sha(prov['board_source'])!=prov['board_sha256']:raise ValueError('Calibration inputs changed: regenerate config')
        if par['bag_path'] in bag_paths:raise ValueError('Repeated bag is not an independent scene')
        bag_paths.add(par['bag_path']);board(prov['board_source'])
        if a.mode!='diagnose':
            review=read(s['review'])
            if review.get('scene')!=s['name'] or str(Path(review.get('bag','')).resolve())!=par['bag_path'] or any(review.get(k) is not True for k in CHECKS) or review.get('reviewer') in (None,'UNCONFIGURED',''):raise ValueError('Complete scene feature review before solving: '+s['name'])
            for k in ['image_diagnostic','lidar_diagnostic']:
                if not Path(review[k]).is_file():raise ValueError('Missing diagnostic: '+k)
        configs.append((s,cfg,par,prov))
    # No historical output directory is ever reused, even on failure.
    root=ROOT/'results'/(a.mode+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));root.mkdir(exist_ok=False)
    save(root/'manifest.json',{'source_manifest':str(Path(a.manifest).resolve()),'sha256':sha(a.manifest),'mode':a.mode,'scenes':scenes})
    blocks=[]
    for s,cfg,par,prov in configs:
        out=root/s['name'];out.mkdir();par={**par,'output_path':str(out),'exit_after_solve':True,'use_point_pick':False};local=out/'qr_params.yaml';local.write_text(yaml.safe_dump(ros_parameter_document(par)))
        shutil.copyfile(str(cfg)+'.provenance.json',out/'provenance.json')
        if a.mode!='diagnose':shutil.copyfile(s['review'],out/'quality_review.yaml')
        r=subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/calib.py'),'audit',par['bag_path'],'--out',str(out/'bag_audit.json')],stdout=(out/'audit.log').open('w'),stderr=subprocess.STDOUT)
        if r.returncode:raise ValueError('Bag integrity check failed: '+str(out))
        run('lidar_diagnostic',local,out/'lidar_diagnostic.log')
        if a.mode=='diagnose':print(out);continue
        run('fast_calib',local,out/'solve.log');L,C=centers(out/'circle_center_record.txt');blocks.append((s['name'],L,C,par))
        quality(out/'single_calib_result.txt',blocks[-1:],par,out/'extrinsics.yaml',True)
    if a.mode=='multi':
        merged=root/'joint';merged.mkdir();text=''.join((root/s['name']/'circle_center_record.txt').read_text() for s in scenes);(merged/'circle_center_record.txt').write_text(text)
        pp={**configs[0][2],'output_path':str(merged)};local=merged/'qr_params.yaml';local.write_text(yaml.safe_dump({'multi_fast_calib':{'ros__parameters':pp}}));run('multi_fast_calib',local,merged/'solve.log');quality(merged/'multi_calib_result.txt',blocks,pp,merged/'extrinsics.yaml',True)
    print('Results:',root,'; geometry acceptance remains false pending independent validation')
if __name__=='__main__':
    try:main()
    except (ValueError,KeyError,FileNotFoundError,subprocess.TimeoutExpired) as e:print('BLOCKED:',e,file=sys.stderr);sys.exit(2)
