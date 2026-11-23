"""Small shared 3D transform helper for exact-stamp observation display."""
import numpy as np

def rotation(q):
    x,y,z,w=np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def observation(data,transform):
    """Project full 3D odom obstacle samples and the observation grid into map."""
    import math
    t=transform['transform'];q=t['rotation'];p=t['translation']
    R=rotation([q['x'],q['y'],q['z'],q['w']])
    offset=np.array([p['x'],p['y'],p['z']])
    source=data['obstacles']
    points=np.array([[v['x'],v['y'],v['z']] for v in source],dtype=float).reshape(-1,3)
    obstacles=(points@R.T+offset)[:,:2].tolist()
    indices=np.flatnonzero(np.asarray(data['observed_cells'])==0)
    grid=np.column_stack(((indices%80+.5)*.05-2,(indices//80+.5)*.05-2,np.zeros(len(indices))))
    sensor=data['sensor_pose'];c,s=math.cos(sensor['theta']),math.sin(sensor['theta'])
    yaw_R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    grid=grid@yaw_R.T+np.array([sensor['x'],sensor['y'],0.])
    coverage=(grid@R.T+offset)[:,:2].tolist()
    from workbench.core import stamp
    return {'stamp_ns':stamp(data),'valid':data['valid'],'obstacles':obstacles,'coverage':coverage,'reason':data['reason']}
