#!/usr/bin/env python3
"""Render real, strictly paired frontend outputs plus recorded camera into H.264.

No ROS nodes, TF, devices, point-cloud registration or estimation are started.
The native frontend cloud is already in IMU coordinates: p_local=R_local_imu*p_imu+t.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import time

import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField


FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
WIDTH, HEIGHT = 1920, 1080
BG, PANEL = (13, 20, 32), (19, 30, 46)  # RGB for Pillow; canvas becomes BGR.
CYAN, GREEN, WHITE = (245, 209, 68), (166, 230, 95), (237, 239, 244)  # BGR.


def db_open(folder):
    files = list(folder.glob('*.db3'))
    if len(files) != 1:raise ValueError('Expected exactly one SQLite database: '+str(folder))
    return sqlite3.connect(files[0].resolve().as_uri()+'?mode=ro', uri=True)


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def rotation(q):
    v = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    norm = np.linalg.norm(v)
    if not np.isfinite(v).all() or norm < 1e-6 or abs(norm-1)>1e-3:
        raise ValueError('Invalid quaternion')
    x,y,z,w = v/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def cloud_xyz(message):
    fields = {f.name:f for f in message.fields}
    if message.is_bigendian or len(message.data) != message.row_step*message.height:
        raise ValueError('Unexpected cloud memory layout')
    if message.row_step < message.point_step*message.width:raise ValueError('Cloud row too short')
    result = []
    for name in ('x','y','z'):
        f = fields[name]
        if f.datatype != PointField.FLOAT32 or f.count != 1 or f.offset+4>message.point_step:
            raise ValueError('Invalid xyz field')
        result.append(np.ndarray((message.height,message.width),dtype='<f4',buffer=message.data,
                      offset=f.offset,strides=(message.row_step,message.point_step)).reshape(-1))
    points = np.column_stack(result)
    return points[np.isfinite(points).all(axis=1)]


def load_frames(native):
    connection = db_open(native)
    ids = {name:tid for tid,name in connection.execute('SELECT id,name FROM topics')}
    poses = {}
    for data, in connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',
                                    (ids['/fast_livo/odometry'],)):
        m = deserialize_message(data, Odometry)
        if m.header.frame_id!='fast_livo_local' or m.child_frame_id!='fast_livo_imu':
            raise ValueError('Unexpected frontend pose frame')
        p = m.pose.pose.position
        if stamp(m) in poses:raise ValueError('Duplicate frontend pose stamp')
        poses[stamp(m)] = (np.array([p.x,p.y,p.z]),rotation(m.pose.pose.orientation))
    frames = []
    seen = set()
    for data, in connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',
                                    (ids['/fast_livo/deskewed_imu'],)):
        m = deserialize_message(data,PointCloud2)
        t = stamp(m)
        if t not in poses or t in seen:raise ValueError('Cloud/pose are not strictly paired')
        if m.header.frame_id!='fast_livo_imu':raise ValueError('Cloud must already be in IMU coordinates')
        position,R = poses[t]
        if not np.isfinite(position).all():raise ValueError('Nonfinite position')
        points = cloud_xyz(m)@R.T+position  # Exactly one local-from-IMU transform.
        # Display voxel sampling only; original native_frames remains unchanged.
        _, indices = np.unique(np.floor(points/.065).astype(np.int64),axis=0,return_index=True)
        frames.append((t,position,R,points[indices].astype(np.float32)))
        seen.add(t)
    connection.close()
    if set(poses)!=seen or len(frames)<2:raise ValueError('Missing pair or insufficient frames')
    if any(b[0]<=a[0] for a,b in zip(frames,frames[1:])):raise ValueError('Timestamp regression')
    return frames


def camera_index(raw):
    connection = db_open(raw)
    tid = connection.execute('SELECT id FROM topics WHERE name=?',('/front/image_sync',)).fetchone()[0]
    rows = []
    for mid,prefix in connection.execute('SELECT id,substr(data,1,16) FROM messages WHERE topic_id=? ORDER BY timestamp,id',(tid,)):
        sec,ns = struct.unpack_from('<iI',prefix,4)
        rows.append((sec*10**9+ns,mid))
    rows.sort()
    if not rows:raise ValueError('No recorded camera images')
    return connection, np.array([r[0] for r in rows],dtype=np.int64), [r[1] for r in rows]


class Renderer:
    def __init__(self, frames, raw, fps, speed):
        self.frames,self.fps,self.speed = frames,fps,speed
        self.times = np.array([f[0] for f in frames],dtype=np.int64)
        self.positions = np.array([f[1] for f in frames])
        self.distance = np.r_[0,np.cumsum(np.linalg.norm(np.diff(self.positions,axis=0),axis=1))]
        # Bounds from all display samples, not from an assumed gravity-aligned world.
        all_points = np.concatenate([f[3] for f in frames])
        self.full_low,self.full_high = np.min(all_points,axis=0),np.max(all_points,axis=0)
        # A declared local viewport, equivalent to zooming a 3D viewer. Distant
        # returns remain in native_frames, but do not set the presentation zoom.
        self.low=self.positions.min(axis=0)-np.array([5.,5.,2.5])
        self.high=self.positions.max(axis=0)+np.array([5.,5.,2.5])
        self.center = (self.low+self.high)/2
        self.radius = max(1.,np.linalg.norm(self.high-self.low)/2)
        visible=all_points[np.all((all_points>=self.low)&(all_points<=self.high),axis=1)]
        if not len(visible):raise ValueError('No points in trajectory-centered viewport')
        self.zlo,self.zhi = np.percentile(visible[:,2],[2,98])
        del all_points
        self.voxels = {};self.last_index = -1;self.last_camera = None;self.last_mid = None
        self.camera_connection,self.image_times,self.image_ids = camera_index(raw)
        self.camera_offsets=[]
        self.base = self.make_base()
        self.fixed_scale = self.project_scale()
        self.xycenter = (self.positions[:,:2].min(axis=0)+self.positions[:,:2].max(axis=0))/2
        extent = np.maximum(np.ptp(self.positions[:,:2],axis=0),.5)
        self.xyscale = min(518/extent[0],165/extent[1])
        print(json.dumps({'pairs':len(frames),'cloud_bounds':[self.low.tolist(),self.high.tolist()],
                          'trajectory_m':self.distance[-1],'radius_m':self.radius}),flush=True)

    def make_base(self):
        im=PILImage.new('RGB',(WIDTH,HEIGHT),BG);draw=ImageDraw.Draw(im)
        font=lambda s:ImageFont.truetype(FONT,s)
        draw.text((36,15),'FAST-LIVO2',font=font(46),fill=(237,244,255))
        draw.text((365,42),'LiDAR · IMU · Camera',font=font(24),fill=(78,213,234))
        draw.text((36,79),'三传感器融合建图实录  /  真实采集数据 · 离线回放',font=font(21),fill=(160,179,200))
        for box in [(30,134,1224,1020),(1250,134,1890,651),(1250,674,1890,1020)]:
            draw.rounded_rectangle(box,radius=15,fill=PANEL,outline=(43,62,84),width=2)
        draw.text((52,150),'3D 局部地图与轨迹',font=font(23),fill=(220,233,245))
        draw.text((1270,145),'采集相机画面',font=font(22),fill=(220,233,245))
        draw.text((1270,683),'轨迹俯视图 · Local XY',font=font(22),fill=(220,233,245))
        draw.text((36,1035),'前端重建展示 · 未应用回环优化',font=font(20),fill=(141,160,181))
        draw.text((1315,1035),'RANGER  /  OFFLINE DEMO',font=font(20),fill=(103,130,155))
        return np.asarray(im)[:,:,::-1].copy()

    @staticmethod
    def basis(yaw=.72):
        elevation=.68
        right=np.array([math.cos(yaw),-math.sin(yaw),0.])
        up=np.array([math.sin(yaw)*math.sin(elevation),math.cos(yaw)*math.sin(elevation),math.cos(elevation)])
        depth=np.cross(right,up)
        return np.column_stack((right,up,depth))

    def project_scale(self):
        corners=np.array([[x,y,z] for x in (self.low[0],self.high[0]) for y in (self.low[1],self.high[1]) for z in (self.low[2],self.high[2])])
        ranges=[]
        for yaw in (.72,.9,1.1):ranges.append(np.ptp((corners-self.center)@self.basis(yaw),axis=0)[:2])
        extent=np.maximum(np.max(ranges,axis=0),1.)
        return min(1090/extent[0],740/extent[1])

    def project(self,points,yaw=.72):
        projected=(points-self.center)@self.basis(yaw)
        uv=np.column_stack((627+projected[:,0]*self.fixed_scale,610-projected[:,1]*self.fixed_scale))
        return uv.astype(np.int32),projected[:,2]

    def colors(self,points):
        z=(points[:,2]-self.zlo)/max(.1,self.zhi-self.zlo)
        values=(np.clip(z,0,1)*255).astype(np.uint8)
        return cv2.applyColorMap(values.reshape(-1,1),cv2.COLORMAP_TURBO).reshape(-1,3)

    def cloud_draw(self,canvas,points,yaw,current=False):
        uv,depth=self.project(points,yaw)
        valid=(uv[:,0]>48)&(uv[:,0]<1205)&(uv[:,1]>199)&(uv[:,1]<999)
        uv,depth=uv[valid],depth[valid]
        colors=self.colors(points[valid])
        if current:colors=np.clip(colors.astype(np.float32)*.7+np.array([55,65,65]),0,255).astype(np.uint8)
        order=np.argsort(depth);uv,colors=uv[order],colors[order]
        for dx,dy in ((0,0),(1,0),(0,1)):
            canvas[uv[:,1]+dy,uv[:,0]+dx]=colors

    def camera(self,time_ns):
        pos=int(np.searchsorted(self.image_times,time_ns))
        choices=[i for i in (pos-1,pos) if 0<=i<len(self.image_ids)]
        index=min(choices,key=lambda i:abs(int(self.image_times[i])-int(time_ns)))
        offset=int(self.image_times[index])-int(time_ns)
        if abs(offset)>150_000_000:raise ValueError('Camera sample more than 150 ms from displayed state')
        self.camera_offsets.append(offset)
        mid=self.image_ids[index]
        if mid!=self.last_mid:
            data=self.camera_connection.execute('SELECT data FROM messages WHERE id=?',(mid,)).fetchone()[0]
            message=deserialize_message(data,Image)
            if message.encoding not in ('rgb8','bgr8'):raise ValueError('Unsupported image encoding')
            image=np.ndarray((message.height,message.width,3),dtype=np.uint8,buffer=message.data,
                             strides=(message.step,3,1))
            if message.encoding=='rgb8':image=image[:,:,::-1]
            ratio=min(614/message.width,452/message.height)
            size=(round(message.width*ratio),round(message.height*ratio))
            scaled=cv2.resize(image,size,interpolation=cv2.INTER_AREA)
            self.last_camera=np.full((452,614,3),(25,33,44),dtype=np.uint8)
            left,top=(614-size[0])//2,(452-size[1])//2
            self.last_camera[top:top+size[1],left:left+size[0]]=scaled
            self.last_mid=mid
        return self.last_camera

    def compose(self,index,yaw=.72,ending=False):
        for i in range(self.last_index+1,index+1):
            points=self.frames[i][3]
            points=points[np.all((points>=self.low)&(points<=self.high),axis=1)]
            keys=np.floor(points/.065).astype(np.int32)
            for key,p in zip(keys,points):self.voxels[tuple(key)]=p
        self.last_index=max(self.last_index,index)
        canvas=self.base.copy()
        cv2.line(canvas,(35,119),(1886,119),(67,91,116),1,cv2.LINE_AA)
        points=np.array(list(self.voxels.values()),dtype=np.float32)
        # A local-coordinate grid; no assumption about physical gravity calibration.
        z=self.low[2];step=max(.5,round(self.radius/5))
        for x in np.arange(math.floor(self.low[0]/step)*step,self.high[0]+step,step):
            uv,_=self.project(np.array([[x,self.low[1],z],[x,self.high[1],z]]),yaw)
            cv2.line(canvas,tuple(uv[0]),tuple(uv[1]),(43,49,58),1,cv2.LINE_AA)
        for y in np.arange(math.floor(self.low[1]/step)*step,self.high[1]+step,step):
            uv,_=self.project(np.array([[self.low[0],y,z],[self.high[0],y,z]]),yaw)
            cv2.line(canvas,tuple(uv[0]),tuple(uv[1]),(43,49,58),1,cv2.LINE_AA)
        self.cloud_draw(canvas,points,yaw)
        self.cloud_draw(canvas,self.frames[index][3],yaw,True)
        path,_=self.project(self.positions[:index+1],yaw)
        cv2.polylines(canvas,[path.reshape(-1,1,2)],False,GREEN,3,cv2.LINE_AA)
        t,p,R,_=self.frames[index]
        mark,_=self.project(np.array([p,p+R[:,0]*.45]),yaw)
        cv2.circle(canvas,tuple(mark[0]),8,(240,245,245),-1,cv2.LINE_AA)
        cv2.circle(canvas,tuple(mark[0]),12,GREEN,2,cv2.LINE_AA)
        cv2.arrowedLine(canvas,tuple(mark[0]),tuple(mark[1]),GREEN,3,cv2.LINE_AA,tipLength=.25)
        canvas[187:639,1263:1877]=self.camera(t)
        # Clip map graphics to their panel by restoring the surrounding layout.
        # The grid may exceed the bounds, so all other panels are restored here.
        canvas[:134]=self.base[:134];canvas[1020:]=self.base[1020:]
        canvas[:,1225:1250]=self.base[:,1225:1250]
        canvas[674:1020,1250:1891]=self.base[674:1020,1250:1891]
        small=self.positions[:index+1,:2]-self.xycenter
        xy=np.column_stack((1570+small[:,0]*self.xyscale,826-small[:,1]*self.xyscale)).astype(np.int32)
        cv2.polylines(canvas,[xy.reshape(-1,1,2)],False,GREEN,3,cv2.LINE_AA)
        cv2.circle(canvas,tuple(xy[0]),5,CYAN,-1,cv2.LINE_AA)
        cv2.circle(canvas,tuple(xy[-1]),7,WHITE,-1,cv2.LINE_AA)
        duration=(self.times[-1]-self.times[0])/1e9;elapsed=(t-self.times[0])/1e9
        text=lambda s,at,scale=.65,color=WHITE:cv2.putText(canvas,s,at,cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
        text('FINAL RECONSTRUCTION' if ending else 'RECONSTRUCTING', (52,210),.55,CYAN)
        text('LOCAL VIEW / 5 m TRAJECTORY MARGIN',(635,173),.48,(166,184,203))
        text(f'{elapsed:05.1f} / {duration:.1f} s', (1510,55),.9)
        text('1x PLAYBACK' if self.speed==1 else f'{self.speed:g}x PLAYBACK',(1670,93),.55,(165,184,205))
        text(f'TRAVEL (EST.)   {self.distance[index]:.1f} m',(1275,958),.65)
        text(f'MAP DISPLAY    {len(self.voxels):,} points',(1275,993),.6,(176,192,208))
        cv2.rectangle(canvas,(52,987),(1204,993),(54,68,84),-1)
        cv2.rectangle(canvas,(52,987),(52+int(1152*elapsed/max(duration,.01)),993),GREEN,-1)
        return canvas


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--native',type=Path,required=True)
    p.add_argument('--raw',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fps',type=int,default=20)
    p.add_argument('--speed',type=float,default=1.)
    args=p.parse_args()
    if args.output.exists():p.error('Output already exists')
    if not 1<=args.fps<=60 or not math.isfinite(args.speed) or args.speed<=0:p.error('Invalid fps/speed')
    started=time.monotonic();cv2.setNumThreads(2)
    frames=load_frames(args.native)
    renderer=Renderer(frames,args.raw,args.fps,args.speed)
    writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*'avc1'),args.fps,(WIDTH,HEIGHT))
    if not writer.isOpened():raise RuntimeError('H.264 encoder is unavailable')
    duration=(frames[-1][0]-frames[0][0])/1e9
    count=math.ceil(duration/args.speed*args.fps)+1
    last_index=-1;last_frame=None;written=0
    try:
        for i in range(count):
            t=frames[0][0]+round(i*args.speed*1e9/args.fps)
            index=min(len(frames)-1,max(0,int(np.searchsorted(renderer.times,t,side='right')-1)))
            if index!=last_index:
                last_frame=renderer.compose(index);last_index=index
            writer.write(last_frame);written+=1
            if i==0:cv2.imwrite(str(args.output.with_name('preview_start.jpg')),last_frame)
            if i==count//2:cv2.imwrite(str(args.output.with_name('preview_mid.jpg')),last_frame)
            if i%200==0:print(f'VIDEO_FRAME {i}/{count}',flush=True)
        # Hold the last camera frame while a virtual camera slowly reveals the map.
        for i in range(6*args.fps):
            last_frame=renderer.compose(len(frames)-1,yaw=.72+.38*i/max(1,6*args.fps-1),ending=True)
            writer.write(last_frame);written+=1
        cv2.imwrite(str(args.output.with_suffix('.jpg')),last_frame,[cv2.IMWRITE_JPEG_QUALITY,95])
    finally:
        writer.release();renderer.camera_connection.close()
    capture=cv2.VideoCapture(str(args.output));decoded=0;first=None;last=None
    while True:
        ok,frame=capture.read()
        if not ok:break
        if first is None:first=frame
        last=frame;decoded+=1
    capture.release()
    if decoded!=written:raise RuntimeError(f'Video decode failed: {decoded}/{written}')
    verification={'source_native':str(args.native.resolve()),'source_raw':str(args.raw.resolve()),
        'paired_frontend_frames':len(frames),'source_duration_s':duration,'playback_speed':args.speed,
        'video_fps':args.fps,'video_frames':written,'decoded_frames':decoded,'video_duration_s':written/args.fps,
        'resolution':[WIDTH,HEIGHT],'codec':'H.264 / AVC in MP4','footer':'Frontend reconstruction; no loop optimization',
        'raw_capture_modified':False,'cloud_transform':'p_local = R_local_imu * p_imu + t_local_imu, once per point',
        'display_voxel_m':.065,'maximum_camera_header_offset_ms':float(np.max(np.abs(renderer.camera_offsets))/1e6),
        'display_viewport_xyz':[renderer.low.tolist(),renderer.high.tolist()],
        'full_native_cloud_bounds_xyz':[renderer.full_low.tolist(),renderer.full_high.tolist()],
        'estimated_travel_m':float(renderer.distance[-1]),'display_points':len(renderer.voxels),
        'physical_sync_proven':False,'ground_truth_accuracy_validated':False,
        'wall_seconds':time.monotonic()-started,'output_bytes':args.output.stat().st_size,
        'sha256':hashlib.sha256(args.output.read_bytes()).hexdigest()}
    args.output.with_suffix('.json').write_text(json.dumps(verification,indent=2)+'\n')
    contact=np.vstack([cv2.resize(first,(960,540)),cv2.resize(last,(960,540))])
    cv2.imwrite(str(args.output.with_name('contact_sheet.jpg')),contact)
    print(json.dumps(verification,indent=2),flush=True)


if __name__=='__main__':main()
