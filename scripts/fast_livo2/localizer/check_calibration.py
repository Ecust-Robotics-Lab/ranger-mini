#!/usr/bin/env python3
"""Held-out static board: fixed extrinsics vs observed hole boundaries, never refit T."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'calibration/lidar_camera_extrinsics/scripts'))
import common as cal
from pcd_diagnostic import pcd


def ellipse_points(ellipse, count=4096):
    center,axes,degrees=ellipse
    theta=np.arange(count)*2*np.pi/count
    local=np.column_stack([np.cos(theta)*axes[0]/2,np.sin(theta)*axes[1]/2])
    a=np.deg2rad(degrees);R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    return local@R.T+center


def radial_error(points,ellipse):
    center,axes,degrees=ellipse;a=np.deg2rad(degrees)
    R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    local=(points-center)@R
    return (np.linalg.norm(local/(np.array(axes)/2),axis=1)-1)*np.mean(axes)/2


def robust_ellipse(points,seed):
    rng=np.random.default_rng(20261104);best=None
    for _ in range(400):
        try:e=cv2.fitEllipse(points[rng.choice(len(points),6,replace=False)].astype(np.float32))
        except cv2.error:continue
        if np.linalg.norm(np.array(e[0])-seed[0])>15 or min(e[1])<min(seed[1])*.7 or max(e[1])>max(seed[1])*1.3:continue
        mask=np.abs(radial_error(points,e))<1.5
        if best is None or mask.sum()>best[0]:best=(int(mask.sum()),e,mask)
    if best is None or best[0]<30:raise ValueError('insufficient observed hole edge support')
    e=best[1]
    for _ in range(3):
        mask=np.abs(radial_error(points,e))<1.5
        e=cv2.fitEllipse(points[mask].astype(np.float32))
    support=points[mask]-np.array(e[0]);bins=len(np.unique(((np.arctan2(support[:,1],support[:,0])+np.pi)/(2*np.pi)*36).astype(int)%36))
    if bins<24:raise ValueError('hole boundary has insufficient angular coverage')
    return e,{'candidate_edge_pixels':len(points),'inliers':int(mask.sum()),'angular_bins_of_36':bins,
              'fit_radial_rms_px':float(np.sqrt(np.mean(radial_error(points[mask],e)**2)))}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    if not args.execute:print('DRY-RUN: held-out static board, fixed T, no hardware');return 0
    output=ROOT/'outputs/fast-livo-apriltag-heldout-calibration-20261104';output.mkdir(exist_ok=False)
    bag=ROOT/'calibration/lidar_camera_extrinsics/datasets/validation_01/20261021_185640_899318/bag'
    run=ROOT/'calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407'
    image,_,info=cal.paired_sample(bag,1)
    cfg=cal.intrinsics(run/'joint/camera_info_used.yaml');K=np.array(cfg['K']);D=np.array(cfg['D'])
    board=cal.board(ROOT/'calibration/lidar_camera_extrinsics/config/board.yaml')
    T=np.array(cal.read(run/'joint/extrinsics.yaml')['T_cam_lidar'])
    marker_corners,ids,_=cv2.aruco.detectMarkers(image,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
    found={int(i):c.reshape(4,2) for i,c in zip(ids.ravel(),marker_corners)}
    obj,holes=cal.board_geometry(board);pixels=np.concatenate([found[i] for i in (1,2,4,3)])
    success,r,t=cv2.solvePnP(np.concatenate(obj),pixels,K,D);assert success
    R=cv2.Rodrigues(r)[0];cb=np.eye(4);cb[:3,:3]=R;cb[:3,3]=t.ravel()
    lb=np.linalg.inv(T)@cb
    outline=np.array([[-.7,-.5,0],[.7,-.5,0],[.7,.5,0],[-.7,.5,0]])
    outlineL=outline@lb[:3,:3].T+lb[:3,3]
    low=outlineL.min(axis=0)-.15;high=outlineL.max(axis=0)+.15
    roi=[float(v) for pair in zip(low,high) for v in pair]
    lidar=output/'lidar';lidar.mkdir()
    exe=ROOT/'calibration/lidar_camera_extrinsics/ws/install_integrated/fast_calib/lib/fast_calib/lidar_diagnostic'
    command=[str(exe),'--offline',str(bag),str(lidar),*map(str,roi),'.12','.5','.4','50']
    with (output/'lidar.log').open('wb') as log:
        done=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,
              env=dict(os.environ,ROS_DOMAIN_ID='194',ROS_LOCALHOST_ONLY='1'),timeout=180)
    if done.returncode:raise RuntimeError('held-out LiDAR feature extraction failed; retained '+str(output))
    H,_=cv2.findHomography(np.concatenate(obj)[:,:2],pixels)
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    edges=cv2.Canny(cv2.GaussianBlur(gray,(3,3),.8),30,90)
    ys,xs=np.nonzero(edges);edgepixels=np.column_stack([xs,ys]).astype(float)
    boardxy=cv2.perspectiveTransform(edgepixels.reshape(-1,1,2),np.linalg.inv(H)).reshape(-1,2)
    fits=[];quality=[]
    theta=np.arange(512)*2*np.pi/512
    for center in holes:
        radius=np.linalg.norm(boardxy-center[:2],axis=1)
        selected=edgepixels[(radius>.095)&(radius<.145)]
        rect=cv2.undistortPoints(selected.reshape(-1,1,2),K,D,P=K).reshape(-1,2)
        ring=center[:2]+.12*np.column_stack([np.cos(theta),np.sin(theta)])
        seedraw=cv2.perspectiveTransform(ring.reshape(-1,1,2),H)
        seedrect=cv2.undistortPoints(seedraw,K,D,P=K).astype(np.float32)
        ellipse,q=robust_ellipse(rect,cv2.fitEllipse(seedrect))
        fits.append(ellipse);quality.append(q)
    centers=pcd(lidar/'centers.pcd');plane=pcd(lidar/'plane.pcd')
    _,_,vt=np.linalg.svd(plane-plane.mean(axis=0),full_matrices=False);u,v=vt[:2]
    centerC=centers@T[:3,:3].T+T[:3,3]
    centerpx=cv2.projectPoints(centerC,np.zeros(3),np.zeros(3),K,np.zeros(5))[0].reshape(-1,2)
    row,col=linear_sum_assignment(np.linalg.norm(centerpx[:,None,:]-np.array([e[0] for e in fits])[None,:,:],axis=2))
    vis=image.copy();results=[]
    for li,im in zip(row,col):
        ring=centers[li]+.12*(np.cos(theta)[:,None]*u+np.sin(theta)[:,None]*v)
        camera=ring@T[:3,:3].T+T[:3,3]
        predicted=cv2.projectPoints(camera,np.zeros(3),np.zeros(3),K,np.zeros(5))[0].reshape(-1,2)
        observed=ellipse_points(fits[im]);distance=cKDTree(observed).query(predicted)[0]
        results.append({'hole_index':int(im),'lidar_center_index':int(li),'curve_distance_rect_px':{
            'p50':float(np.median(distance)),'p95':float(np.percentile(distance,95)),
            'rms':float(np.sqrt(np.mean(distance**2))),'max':float(distance.max())},'image_edge_fit':quality[im]})
        for points,color in [(predicted,(0,0,255)),(observed[::8],(0,255,0))]:
            ray=np.column_stack([(points[:,0]-K[0,2])/K[0,0],(points[:,1]-K[1,2])/K[1,1],np.ones(len(points))])
            raw=cv2.projectPoints(ray,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
            cv2.polylines(vis,[np.rint(raw).astype(np.int32)],True,color,1)
    cv2.imwrite(str(output/'hole_boundary_overlay.png'),vis)
    result={'kind':'held_out_observed_hole_boundary_diagnostic','bag':str(bag),'input_pair':info,
        'T_cam_lidar_file':str(run/'joint/extrinsics.yaml'),'T_cam_lidar_sha256':cal.sha(run/'joint/extrinsics.yaml'),
        'T_cam_lidar_refitted':False,'roi_from_image_pose_with_15cm_margin_only':roi,'lidar_argv':command,
        'image_ellipse_fit_domain':'undistorted pixels, K original, zero D; raw overlay redistorted',
        'hole_results':results,'geometry_accepted':False,'runtime_modified':False,
        'limitations':['Observed image edges, not ArUco-inferred hole centers; marker homography only seeds broad ROIs',
            'LiDAR circle centers/plane and nominal measured 0.12 m radius have unquantified errors',
            'Nearest sampled ellipse distance is an internal cross-sensor check, not absolute calibration accuracy',
            'One independent static board pose, no dynamic synchronization evidence']}
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
