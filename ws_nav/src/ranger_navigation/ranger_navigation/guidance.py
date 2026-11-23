"""Trim guidance by the same observation envelope used by the controller/gate."""
import math
import numpy as np
from .geometry import wrap

def observed_prefix(path,pose,correction,observer,grid,body,policy,max_distance=.75):
    path=np.asarray(path)
    nearest=int(np.argmin(np.linalg.norm(path[:,:2]-pose[:2],axis=1)+.15*np.abs(wrap(path[:,2]-pose[2]))))
    c,s=math.cos(correction[2]),math.sin(correction[2]);map_to_odom=np.array([[c,-s],[s,c]])
    c,s=math.cos(observer[2]),math.sin(observer[2]);odom_to_grid=np.array([[c,-s],[s,c]])
    def check(p):
        xy=((p[:2]-correction[:2])@map_to_odom-observer[:2])@odom_to_grid
        return grid.coverage_report([*xy,p[2]-correction[2]-observer[2]],body,policy)
    diagnostic={'current':check(np.asarray(pose)),'blocked':None,'checked_distance_m':0.}
    if not diagnostic['current']['accepted']:
        return None,diagnostic
    known_end=nearest;distance=0.
    for i in range(nearest,len(path)):
        if i>nearest:distance+=np.linalg.norm(path[i,:2]-path[i-1,:2])
        if distance>max_distance:break
        report=check(path[i])
        if not report['accepted']:
            diagnostic['blocked']={**report,'path_index':i,'distance_m':float(distance)};break
        known_end=i;diagnostic['checked_distance_m']=float(distance)
    if known_end<=nearest and nearest<len(path)-1:return None,diagnostic
    # Preserve the original anchor for a fixed parallel steering axis.
    return path[:known_end+1],diagnostic
