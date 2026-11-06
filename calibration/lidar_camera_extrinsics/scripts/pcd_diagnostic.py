#!/usr/bin/python3
"""Render actual PCL diagnostic clouds without GUI; never solves extrinsics."""
import argparse,sys
import numpy as np
from common import *
def pcd(path):
    with Path(path).open('rb') as f:
        h={}
        while True:
            line=f.readline().decode('ascii').strip()
            if not line:continue
            x=line.split();h[x[0]]=x[1:]
            if x[0]=='DATA':break
        names=h['FIELDS'];count=list(map(int,h.get('COUNT',['1']*len(names))));sizes=list(map(int,h['SIZE']));types=h['TYPE'];n=int(h['POINTS'][0])
        if h['DATA'][0]=='ascii':arr=np.loadtxt(f,ndmin=2);return arr[:,[names.index(k) for k in ('x','y','z')]]
        if h['DATA'][0]!='binary' or any(c!=1 for c in count):raise ValueError('Unsupported PCD layout')
        dtype=np.dtype([(k,{'F':'f','U':'u','I':'i'}[t]+str(s)) for k,t,s in zip(names,types,sizes)])
        a=np.frombuffer(f.read(n*dtype.itemsize),dtype=dtype,count=n);return np.column_stack([a[k] for k in ('x','y','z')])
def main():
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--board',required=True);a=p.parse_args();b=board(a.board,require_confirm=False);out=output(a.directory)
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(12,6));ax=fig.add_subplot(121,projection='3d');ax2=fig.add_subplot(122);stats=[]
    roi=pcd(out/'roi.pcd');q=roi[::max(1,len(roi)//30000)];ax.scatter(*q.T,s=1);ax.set_xlabel('LiDAR x m');ax.set_ylabel('y m');ax.set_zlabel('z m')
    edge=pcd(out/'edges.pcd');ax2.scatter(edge[:,0],edge[:,1],s=2);ax2.set_aspect('equal');ax2.set_xlabel('Flattened plane x m');ax2.set_ylabel('y m')
    candidates=pcd(out/'circle_candidates.pcd') if (out/'circle_candidates.pcd').exists() else []
    for i,c in enumerate(candidates):
        delta=edge[:,:2]-c[:2];error=np.abs(np.linalg.norm(delta,axis=1)-b['circle_radius']);mask=error<.02;angles=np.arctan2(delta[mask,1],delta[mask,0]);bins=np.unique(np.floor((angles+np.pi)/(2*np.pi)*24).astype(int)%24)
        stats.append({'candidate':i,'center_flattened_m':c.tolist(),'boundary_support_within_2cm':int(mask.sum()),'occupied_angular_bins_of_24':len(bins),'nominal_radius_rms_m':float(np.sqrt(np.mean(error[mask]**2))) if mask.any() else None})
        ax2.add_patch(plt.Circle(c[:2],b['circle_radius'],fill=False));ax2.text(c[0],c[1],str(i))
    fig.tight_layout();fig.savefig(out/'lidar_diagnostic.png',dpi=160);save(out/'circle_support.json',{'candidates':stats,'note':'Nominal-radius support is diagnostic; candidate selection/physical hole identity must be reviewed','geometry_accepted':False});print(out/'lidar_diagnostic.png')
if __name__=='__main__':main()
