#!/usr/bin/python3
"""Feature overlays and leave-one-scene-out diagnostics; does not accept geometry or integrate."""
import argparse,itertools
from common import *
from pcd_diagnostic import pcd
from scipy.spatial.transform import Rotation
import cv2,numpy as np

def fit(L,C):
 ml=L.mean(0);mc=C.mean(0);u,s,vt=np.linalg.svd((L-ml).T@(C-mc));v=vt.T;D=np.eye(3);D[2,2]=np.linalg.det(v@u.T);R=v@D@u.T
 if s[1]<1e-10:raise ValueError('Degenerate centers')
 T=np.eye(4);T[:3,:3]=R;T[:3,3]=mc-R@ml;matrix_check(T);return T

def main():
 p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();run=output(a.run);joint=read(run/'joint/extrinsics.yaml');Tj=np.array(joint['T_cam_lidar']);scenes=joint['scenes'];L=[np.array(s['lidar_centers']) for s in scenes];C=[np.array(s['camera_centers']) for s in scenes];summary={'joint_fit':joint,'single_scenes':[],'leave_one_scene_out':[],'pairwise_single_transform_changes':[],'geometric_calibration_accepted':False,'independent_ground_truth_available':False}
 for k,s in enumerate(scenes):
  name=s['name'];par=read(run/name/'qr_params.yaml')['fast_calib']['ros__parameters'];intr=intrinsics(read(run/name/'provenance.json')['intrinsics']['source']);K=np.array(intr['K']);D=np.array(intr['D']);single=read(run/name/'extrinsics.yaml');T=np.array(single['T_cam_lidar'])
  im,_,info=sample(par['bag_path'],1);vis=im.copy();diag=run/name
  orig=pcd(diag/'plane.pcd');flat=pcd(diag/'aligned.pcd');edge=pcd(diag/'edges.pcd');A=np.linalg.lstsq(orig,flat[:,:2],rcond=None)[0];Rflat=np.stack([A[:,0],A[:,1],np.cross(A[:,0],A[:,1])]);avg=(orig@Rflat[2]).mean();edgeL=np.c_[edge[:,:2],np.full(len(edge),avg)]@np.linalg.inv(Rflat).T
  edgeC=(Tj[:3,:3]@edgeL.T).T+Tj[:3,3];good=edgeC[:,2]>1e-4;uv=cv2.projectPoints(edgeC[good],np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
  mask=np.isfinite(uv).all(1)&(uv[:,0]>=0)&(uv[:,0]<im.shape[1])&(uv[:,1]>=0)&(uv[:,1]<im.shape[0])
  for u,v in uv[mask]:cv2.circle(vis,(round(u),round(v)),1,(255,255,0),-1)
  pred=(Tj[:3,:3]@L[k].T).T+Tj[:3,3];pred_uv=cv2.projectPoints(pred,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2);cam_uv=cv2.projectPoints(C[k],np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
  for i,(pu,cu) in enumerate(zip(pred_uv,cam_uv)):
   pp=tuple(np.rint(pu).astype(int));cp=tuple(np.rint(cu).astype(int));cv2.circle(vis,cp,6,(0,255,0),2);cv2.drawMarker(vis,pp,(0,0,255),cv2.MARKER_CROSS,12,2);cv2.line(vis,pp,cp,(0,0,255),1);cv2.putText(vis,str(i),tuple(np.rint(cu+[8,-8]).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
  cv2.rectangle(vis,(0,im.shape[0]-62),(im.shape[1],im.shape[0]),(25,25,25),-1);cv2.putText(vis,'Cyan: LiDAR plane edges | Red: transformed LiDAR centers',(15,im.shape[0]-38),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1);cv2.putText(vis,'Green: centers inferred from ArUco (not independent ground truth)',(15,im.shape[0]-15),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
  out=run/'validation'/name;out.mkdir(exist_ok=True,parents=True);cv2.imwrite(str(out/'feature_overlay.png'),vis)
  entry={'scene':name,'single_fit_rmse_m':single['joint_rmse_m'],'joint_fit_rmse_m':s['rmse_m'],'joint_derived_hole_reprojection_rmse_px':s['derived_hole_reprojection_rmse_px'],'rotation_vs_joint_deg':float(Rotation.from_matrix(T[:3,:3]@Tj[:3,:3].T).magnitude()*180/np.pi),'translation_vs_joint_m':float(np.linalg.norm(T[:3,3]-Tj[:3,3])),'feature_overlay':str(out/'feature_overlay.png')};summary['single_scenes'].append(entry)
  train=[j for j in range(len(scenes)) if j!=k];loo=fit(np.concatenate([L[j] for j in train]),np.concatenate([C[j] for j in train]));pred=(loo[:3,:3]@L[k].T).T+loo[:3,3];err=np.linalg.norm(pred-C[k],axis=1);puv=cv2.projectPoints(pred,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2);px=np.linalg.norm(puv-cam_uv,axis=1)
  summary['leave_one_scene_out'].append({'held_out_scene':name,'training_scenes':[scenes[j]['name'] for j in train],'T_cam_lidar':loo.tolist(),'held_out_residual_m_per_hole':err.tolist(),'held_out_rmse_m':float(np.sqrt(np.mean(err**2))),'held_out_derived_hole_reprojection_rmse_px':float(np.sqrt(np.mean(px**2))),'note':'Held out from rigid-transform fitting only; shares target and camera calibration, not independent ground truth'})
 for i,j in itertools.combinations(range(len(scenes)),2):
  Ti=np.array(read(run/scenes[i]['name']/'extrinsics.yaml')['T_cam_lidar']);Tj2=np.array(read(run/scenes[j]['name']/'extrinsics.yaml')['T_cam_lidar']);summary['pairwise_single_transform_changes'].append({'scenes':[scenes[i]['name'],scenes[j]['name']],'rotation_difference_deg':float(Rotation.from_matrix(Ti[:3,:3]@Tj2[:3,:3].T).magnitude()*180/np.pi),'translation_difference_m':float(np.linalg.norm(Ti[:3,3]-Tj2[:3,3]))})
 save(run/'validation/trial_validation_summary.json',summary);print(json.dumps({k:summary[k] for k in ['single_scenes','leave_one_scene_out','pairwise_single_transform_changes']},indent=2))
if __name__=='__main__':main()
