#!/usr/bin/python3
"""Read-only all-image marker/stability checks and wide cloud overview, no extrinsic solve."""
import argparse,csv,json
import cv2,numpy as np
from common import *
def main():
 p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('--out',required=True);a=p.parse_args();out=output(a.out);out.mkdir(exist_ok=True,parents=True)
 stamps=[];tracks={i:[] for i in [1,2,3,4]};full=0;pts=[];n=0
 detector=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
 for topic,msg,ts in messages(a.bag):
  if topic==LIDAR and n<50:pts.append(xyz(msg));n+=1
  if topic==IMAGE:
   stamps.append(stamp(msg));corners,ids,_=cv2.aruco.detectMarkers(bgr(msg),detector)
   if ids is not None:
    if sorted(ids.ravel().tolist())==[1,2,3,4]:full+=1
    for c,i in zip(corners,ids.ravel()):
     if int(i) in tracks:tracks[int(i)].append(c.reshape(4,2).mean(0))
 if not stamps or not pts:raise ValueError('Missing image or cloud')
 d={'image_frames':len(stamps),'all_four_detected_frames':full,'marker_center_peak_to_peak_px':{str(k):np.ptp(v,axis=0).tolist() if v else None for k,v in tracks.items()},'note':'Image motion metric and driver CSV checks are not physical trigger-edge measurements'}
 state=ROOT/'logs/stage_b_state.json'
 if state.exists():
  csvpath=Path(read(state)['log_dir'])/'camera_sync.csv'
  if csvpath.exists():
   ss=set(stamps)
   with csvpath.open() as f:rd=csv.DictReader(f);fields=rd.fieldnames;rows=[r for r in rd if int(r['ts_low_ns']) in ss]
   fids=[int(r['frame_id']) for r in rows]
   d['camera_driver_csv']={'source':str(csvpath),'matching_rows':len(rows),'all_status_OK':bool(rows) and all(r['status']=='OK' for r in rows),'frameid_first':fids[0] if fids else None,'frameid_last':fids[-1] if fids else None,'frameid_nonunit_steps':int(sum(np.diff(fids)!=1))}
   with (out/'camera_sync_capture_rows.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
 points=np.concatenate(pts);np.save(out/'accumulated_50frames.npy',points)
 q=points[(points[:,0]>.3)&(points[:,0]<4)&(abs(points[:,1])<2)&(abs(points[:,2])<1.5)];q=q[::max(1,len(q)//100000)]
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,axs=plt.subplots(1,3,figsize=(17,6))
 for ax,(i,j,title) in zip(axs,[(1,2,'Front: y vs z, color=x'),(0,1,'Top: x vs y'),(0,2,'Side: x vs z')]):
  sc=ax.scatter(q[:,i],q[:,j],s=.3,c=q[:,0],vmin=.3,vmax=4,cmap='turbo');ax.set_xlabel('xyz'[i]+' (m)');ax.set_ylabel('xyz'[j]+' (m)');ax.set_title(title);ax.set_aspect('equal');ax.grid(True)
 fig.colorbar(sc,ax=axs,label='LiDAR x (m)');fig.savefig(out/'cloud_overview.png',dpi=160);plt.close(fig)
 d['cloud_frames_accumulated']=n;d['cloud_points']=len(points);save(out/'stability_and_driver_csv.json',d);print(json.dumps(d,indent=2))
if __name__=='__main__':main()
