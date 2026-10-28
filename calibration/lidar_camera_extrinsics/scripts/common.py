"""Offline C2 adapters. Original messages and bags are never rewritten."""
from pathlib import Path
import hashlib, json, math, os
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'logs/matplotlib'))
LIDAR='/livox/lidar'; IMAGE='/front/image_sync'
DIMS=['marker_size','delta_width_qr_center','delta_height_qr_center','delta_width_circles','delta_height_circles','circle_radius']
CALIBRATION_ROOT = ROOT.parent
LEGACY_CALIBRATION_ROOT = Path('/home/lab307/calibration')
def relocated(value):
    # Interpret historical absolute paths without rewriting evidence/config bytes.
    if isinstance(value, str) and (value == str(LEGACY_CALIBRATION_ROOT) or value.startswith(str(LEGACY_CALIBRATION_ROOT) + '/')):
        return str(CALIBRATION_ROOT / Path(value).relative_to(LEGACY_CALIBRATION_ROOT))
    if isinstance(value, dict): return {k:relocated(v) for k,v in value.items()}
    if isinstance(value, list): return [relocated(v) for v in value]
    return value
def asset_path(value): return Path(relocated(str(value))).expanduser()
def read(p):
    return relocated(yaml.safe_load(asset_path(p).read_text()))
def output(p):
    p=asset_path(p).resolve()
    if not p.is_relative_to(ROOT): raise ValueError('Outputs must be inside '+str(ROOT))
    p.parent.mkdir(parents=True,exist_ok=True)
    return p
def save(p,d):
    output(p).write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
def sha(p): return hashlib.sha256(asset_path(p).read_bytes()).hexdigest()
def finite(v,name):
    if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v): raise ValueError('Invalid/unmeasured '+name)
    return float(v)
def intrinsics(path):
    src=asset_path(path).resolve();d=read(src)
    def values(x):return x['data'] if isinstance(x,dict) else x
    K=np.array(values(d['camera_matrix']),float).reshape(3,3)
    D=np.array(values(d['distortion_coefficients']),float).ravel()
    w=int(d['image_width']);h=int(d['image_height']);model=d['distortion_model']
    if model!='plumb_bob' or len(D) not in (4,5):raise ValueError('Require OpenCV plumb_bob 4/5 coefficients; fisheye/rational need explicit port')
    if len(D)==4:D=np.r_[D,0.0]
    if not np.isfinite(K).all() or not np.isfinite(D).all() or min(w,h)<=0 or min(K[0,0],K[1,1])<=0 or not 0<=K[0,2]<w or not 0<=K[1,2]<h or not np.allclose(K[2],[0,0,1]) or abs(K[0,1])+abs(K[1,0])>1e-10:raise ValueError('Invalid camera matrix/resolution')
    return {'source':str(src),'sha256':sha(src),'image_width':w,'image_height':h,'distortion_model':model,'K':K.tolist(),'D':D.tolist(),'status':d.get('status','UNSPECIFIED'),'warnings':d.get('warnings',[]),'coefficient_order':['k1','k2','p1','p2','k3']}
def board(path,require_confirm=True):
    b=read(path)
    if b.get('units')!='m' or b.get('dictionary')!='DICT_6X6_250' or b.get('ids_tl_tr_br_bl')!=[1,2,4,3]:raise ValueError('Board units/dictionary/IDs mismatch')
    if require_confirm and not (b.get('measurements_confirmed') is True and b.get('orientation_verified') is True):raise ValueError('Board measurements/orientation not confirmed')
    for k in DIMS:
        if finite(b[k],k)<=0:raise ValueError('Nonpositive '+k)
    if min(b['delta_width_circles'],b['delta_height_circles'])<=2*b['circle_radius']:raise ValueError('Overlapping holes')
    return b
def stamp(m):return int(m.header.stamp.sec)*1000000000+int(m.header.stamp.nanosec)
def messages(path,topics=(LIDAR,IMAGE)):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    path=asset_path(path).resolve()
    if path.is_dir():
        meta=read(path/'metadata.yaml')['rosbag2_bagfile_information'];storage=meta['storage_identifier']
    else: storage={'.db3':'sqlite3','.mcap':'mcap'}.get(path.suffix)
    if storage not in rosbag2_py.get_registered_readers():raise ValueError('Storage plugin unavailable: '+str(storage))
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(path),storage_id=storage),rosbag2_py.ConverterOptions('cdr','cdr'))
    types={t.name:t.type for t in r.get_all_topics_and_types()}
    for topic,typ in [(LIDAR,'sensor_msgs/msg/PointCloud2'),(IMAGE,'sensor_msgs/msg/Image')]:
        if topic in topics and types.get(topic)!=typ:raise ValueError('Missing/wrong topic type '+topic)
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        t,data,ts=r.read_next();yield t,deserialize_message(data,get_message(types[t])),ts

def xyz(m):
    fs={f.name:f for f in m.fields}
    for k in ('x','y','z'):
        if k not in fs or fs[k].datatype!=7 or fs[k].count!=1:raise ValueError('Require FLOAT32 XYZ')
    endian='>' if m.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[fs[k].offset for k in ('x','y','z')],'itemsize':m.point_step})
    if len(m.data)!=m.height*m.row_step:raise ValueError('Malformed PointCloud2 size')
    arr=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
    pts=np.column_stack([arr[k].ravel() for k in ('x','y','z')]);return pts[np.isfinite(pts).all(axis=1)&(np.linalg.norm(pts,axis=1)>1e-4)]
def bgr(m):
    from cv_bridge import CvBridge
    return CvBridge().imgmsg_to_cv2(m,desired_encoding='bgr8')
def sample(path,frames=50):
    pts=[];image=None;info={};n=0
    for t,m,ts in messages(path):
        if t==IMAGE and image is None:image=bgr(m);info['image_stamp_ns']=stamp(m);info['camera_frame']=m.header.frame_id
        elif t==LIDAR and n<frames:
            pts.append(xyz(m));n+=1;info['lidar_frame']=m.header.frame_id
            info.setdefault('lidar_first_stamp_ns',stamp(m));info['lidar_last_stamp_ns']=stamp(m)
        if n>=frames and image is not None:break
    if not pts or image is None:raise ValueError('Missing image/cloud')
    info['accumulated_frames']=n
    return image,np.concatenate(pts),info
def board_geometry(b):
    corners=[];holes=[]
    for sx,sy in [(-1,1),(1,1),(1,-1),(-1,-1)]:
        x=sx*b['delta_width_qr_center'];y=sy*b['delta_height_qr_center'];a=b['marker_size']/2
        corners.append(np.array([[x-a,y+a,0],[x+a,y+a,0],[x+a,y-a,0],[x-a,y-a,0]],np.float32))
        holes.append([sx*b['delta_width_circles']/2,sy*b['delta_height_circles']/2,0])
    return corners,np.array(holes,float)
def matrix_check(T):
    T=np.asarray(T,float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):raise ValueError('Invalid T')
    R=T[:3,:3];det=float(np.linalg.det(R));orth=float(np.linalg.norm(R.T@R-np.eye(3)))
    if abs(det-1)>1e-4 or orth>1e-4:raise ValueError('Rotation invalid')
    inv=np.linalg.inv(T)
    return {'T_cam_lidar':T.tolist(),'T_lidar_cam':inv.tolist(),'rotation_determinant':det,'rotation_orthogonality_error':orth,'inverse_error':float(np.linalg.norm(T@inv-np.eye(4)))}

def ros_parameter_document(params, names=('fast_calib','multi_fast_calib')):
    # Humble rcl YAML parser rejects anchors/aliases. Each node gets its own map.
    return {name:{'ros__parameters':dict(params)} for name in names}

def paired_sample(path,frames=1):
    """Select exact header-matched Image/PointCloud2 pairs without rewriting the bag."""
    if not 1<=frames<=100:raise ValueError('Use 1..100 frames for paired projection')
    clouds={};images={};points=[];paired=[];first_image=None;info={}
    for topic,msg,_ in messages(path):
        ns=stamp(msg)
        if topic==LIDAR:clouds[ns]=msg
        elif topic==IMAGE:images[ns]=msg
        for ns in sorted(clouds.keys() & images.keys()):
            cloud=clouds.pop(ns);im=images.pop(ns)
            if first_image is None:
                first_image=bgr(im)
                info={'image_stamp_ns':ns,'camera_frame':im.header.frame_id,'lidar_frame':cloud.header.frame_id,
                      'skipped_unpaired_images_before_first_pair':len([t for t in images if t<ns]),
                      'skipped_unpaired_clouds_before_first_pair':len([t for t in clouds if t<ns])}
            if cloud.header.frame_id!=info['lidar_frame'] or im.header.frame_id!=info['camera_frame']:raise ValueError('Changing frame IDs')
            points.append(xyz(cloud));paired.append(ns)
            if len(points)>=frames:
                info.update(lidar_first_stamp_ns=paired[0],lidar_last_stamp_ns=paired[-1],accumulated_frames=len(points),exact_header_paired=True,paired_header_stamps_ns=paired)
                return first_image,np.concatenate(points),info
    raise ValueError('Not enough exact image/cloud header pairs for requested frames')
