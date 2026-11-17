#!/usr/bin/env python3
"""Redraw recorded B graph snapshots, online poses and RGB. No ROS nodes/devices.

Graph geometry comes from recorded optimized_path messages, never final poses
backfilled into earlier frames. RGB uses the acquisition's explicit +0.1 s
image phase, once, for software time association only.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image as RosImage

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/fast_livo2'))
import render_demo as rd
sys.path.insert(0, str(Path(__file__).resolve().parent))
import analysis as a

W, H = 1920, 1080
BG = (12, 18, 29)
WHITE, CYAN, GREEN, AMBER = (238, 244, 250), (65, 211, 235), (112, 225, 157), (255, 193, 92)
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'


def load(path):
    return json.loads(Path(path).read_text())


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def pose(m):
    t = np.eye(4)
    t[:3, :3] = rd.rotation(m.orientation)
    t[:3, 3] = [m.position.x, m.position.y, m.position.z]
    return t


def pcd(path):
    with Path(path).open('rb') as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError('Incomplete PCD')
            header.append(line)
            if line.startswith(b'DATA '):
                break
        fields = next(x for x in header if x.startswith(b'FIELDS ')).split()[1:]
        if fields != [b'x', b'y', b'z', b'intensity'] or line.strip() != b'DATA binary':
            raise ValueError('Expected binary XYZI PCD')
        out = np.frombuffer(f.read(), dtype='<f4').reshape(-1, 4)[:, :3].copy()
    if not np.isfinite(out).all():
        raise ValueError('Nonfinite PCD')
    return out


class Data:
    def __init__(self, trial):
        self.trial = trial
        self.capture = trial / 'capture'
        self.events = load(trial / 'events.json')
        self.manifest = load(self.capture / 'map/manifest.json')
        committed = load(self.capture / 'map/COMMITTED')
        assert committed['manifest_sha256'] == digest(self.capture / 'map/manifest.json')
        self.metrics = load(Path(load(trial / 'latest_analysis.json')['directory']) / 'metrics.json')
        self.anchor = load(self.capture / 'clock_anchor.json')['anchor']
        self.begin, self.end = self.events['start_window'][0], self.events['end_hold_complete_ns']
        self.depart, self.returned = self.events['route_start_ns'], self.events['returned_ns']
        self.inv_base = np.linalg.inv(a.base_extrinsic(load(trial / 'calibration.json')))
        self.frames = self.manifest['keyframes']
        self.clouds = []
        for k in self.frames:
            rel = str(Path(k['directory']) / 'full.pcd')
            path = self.capture / 'map' / rel
            assert digest(path) == self.manifest['files'][rel]
            points = pcd(path)
            _, inds = np.unique(np.floor(points / .085).astype(np.int32), axis=0, return_index=True)
            self.clouds.append(points[inds])
        self.connection = rd.db_open(self.capture / 'raw')
        self.topic = dict((n, i) for i, n in self.connection.execute('SELECT id,name FROM topics'))
        self.odom = {}
        for wall, blob in self.rows('/fast_livo/odometry'):
            m = deserialize_message(blob, Odometry)
            assert m.header.frame_id == 'fast_livo_local' and m.child_frame_id == 'fast_livo_imu'
            self.odom[rd.stamp(m)] = pose(m.pose.pose)
        self.odom_times = np.array(sorted(self.odom), dtype=np.int64)
        self.front = np.array([self.odom[t] @ self.inv_base for t in self.odom_times])
        clocks = []
        for wall, blob in self.rows('/clock'):
            m = deserialize_message(blob, Clock)
            clocks.append((wall, m.clock.sec * 10**9 + m.clock.nanosec))
        self.clock_wall = [x[0] for x in clocks]
        self.clock_sensor = [x[1] for x in clocks]
        self.snapshots = []
        for wall, blob in self.rows('/fast_livo/backend/optimized_path'):
            m = deserialize_message(blob, RosPath)
            assert m.header.frame_id == 'map'
            matrices = np.array([pose(p.pose) for p in m.poses])
            assert [rd.stamp(p) for p in m.poses] == [k['stamp_ns'] for k in self.frames[:len(m.poses)]]
            clock_index = bisect.bisect_right(self.clock_wall, wall) - 1
            assert clock_index >= 0
            sensor = self.clock_sensor[clock_index] + wall - self.clock_wall[clock_index]
            self.snapshots.append((self.steady(sensor), matrices))
        self.snapshot_times = [x[0] for x in self.snapshots]
        final = np.array([a.matrix(k['X_map_imu']) for k in self.frames])
        self.final_snapshot_error = float(np.max(np.abs(self.snapshots[-1][1] - final)))
        assert self.final_snapshot_error < 1e-5
        self.observer = [r for r in a.jsonl(self.capture / 'closure_samples.jsonl')
                         if r['kind'] == 'online' and r.get('session_id') == self.manifest['session_id']
                         and r.get('sample_steady_ns') is not None]
        self.observer.sort(key=lambda r: r['received_steady_ns'])
        self.observer_times = [r['received_steady_ns'] for r in self.observer]
        self.online = a.online_rows(self.observer, load(trial / 'config.json'))
        self.online.sort(key=lambda r: r['received_steady_ns'])
        self.online_times = [r['received_steady_ns'] for r in self.online]
        self.online_positions = np.array([(a.matrix(r['pose']) @ self.inv_base)[:3, 3] for r in self.online])
        self.camera_connection, times, self.image_ids = rd.camera_index(self.capture / 'raw')
        self.image_times = times + 100_000_000
        self.last_camera_id, self.last_camera = None, None
        self.camera_offsets = []
        self.last_snapshot = -2
        self.map_points = np.empty((0, 3), dtype=np.float32)
        self.map_matrices = np.empty((0, 4, 4))
        points = np.concatenate([c @ t[:3, :3].T + t[:3, 3] for c, t in zip(self.clouds, final)])
        self.low = np.percentile(points, 1, axis=0) - [.6, .4, .15]
        self.high = np.percentile(points, 99, axis=0) + [.6, .4, .15]
        self.zrange = np.percentile(points[:, 2], [2, 98])
        self.final_points = points
        self.bounds_clipped = True
        print(json.dumps({'keyframes': len(self.frames), 'recorded_snapshots': len(self.snapshots),
                          'online_samples': len(self.online), 'final_snapshot_matrix_error': self.final_snapshot_error,
                          'seconds': (self.end - self.begin)/1e9, 'display_bounds': [self.low.tolist(), self.high.tolist()]}), flush=True)

    def rows(self, topic):
        return self.connection.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',
                                       (self.topic[topic],))

    def sensor(self, steady):
        return int(steady) - self.anchor[1] + self.anchor[0]

    def steady(self, sensor):
        return int(sensor) - self.anchor[0] + self.anchor[1]

    def map_at(self, steady):
        index = bisect.bisect_right(self.snapshot_times, steady) - 1
        if index != self.last_snapshot:
            self.map_matrices = self.snapshots[index][1] if index >= 0 else np.empty((0, 4, 4))
            if len(self.map_matrices):
                pts = np.concatenate([c @ t[:3, :3].T + t[:3, 3]
                                      for c, t in zip(self.clouds, self.map_matrices)])
                pts = pts[np.all((pts >= self.low) & (pts <= self.high), axis=1)]
                _, inds = np.unique(np.floor(pts/.085).astype(np.int32), axis=0, return_index=True)
                self.map_points = pts[inds]
            self.last_snapshot = index
        return self.map_points, self.map_matrices

    def camera(self, sensor):
        idx = int(np.searchsorted(self.image_times, sensor))
        choices = [i for i in (idx-1, idx) if 0 <= i < len(self.image_ids)]
        idx = min(choices, key=lambda i: abs(int(self.image_times[i])-int(sensor)))
        offset = int(self.image_times[idx])-int(sensor)
        if abs(offset) > 150_000_000:
            raise ValueError('RGB effective stamp differs by >150ms')
        self.camera_offsets.append(offset)
        mid = self.image_ids[idx]
        if mid != self.last_camera_id:
            blob = self.camera_connection.execute('SELECT data FROM messages WHERE id=?', (mid,)).fetchone()[0]
            m = deserialize_message(blob, RosImage)
            assert m.encoding in ('rgb8', 'bgr8')
            im = np.ndarray((m.height, m.width, 3), dtype=np.uint8, buffer=m.data, strides=(m.step, 3, 1))
            if m.encoding == 'rgb8':
                im = im[:, :, ::-1]
            self.last_camera = cv2.resize(im, (600, 480), interpolation=cv2.INTER_AREA)
            self.last_camera_id = mid
        return self.last_camera


class Renderer:
    def __init__(self, data):
        self.d = data
        self.fonts = {}
        self.center = (data.low + data.high) / 2
        self.xycenter = (data.low[:2] + data.high[:2]) / 2
        self.xyscale = min(565/(data.high[0]-data.low[0]), 190/(data.high[1]-data.low[1]))
        self.base = Image.new('RGB', (W, H), BG)
        draw = ImageDraw.Draw(self.base)
        for box in ((30, 125, 1245, 985), (1265, 125, 1890, 688), (1265, 708, 1890, 985)):
            draw.rounded_rectangle(box, radius=18, fill=(18, 29, 44), outline=(38, 56, 76), width=2)
        self.text(draw, (35, 16), 'RANGER  /  第 4 轮闭环建图', 36, WHITE, True)
        self.text(draw, (36, 71), 'FAST-LIVO2 三融合前端  +  独立 B 回环后端', 22, CYAN)
        self.text(draw, (53, 143), '三维地图逐步生成', 25, WHITE, True)
        self.text(draw, (53, 180), '按当时记录的优化位姿重建关键帧地图', 18, (149, 168, 190))
        self.text(draw, (1283, 143), '同步 RGB  /  车头相机', 23, WHITE, True)
        self.text(draw, (1283, 720), '实时位姿与轨迹  /  Map XY', 23, WHITE, True)
        self.text(draw, (36, 1039), '真实采集数据离线重绘 · B 在线建图记录 · 无独立位姿真值', 18, (139, 158, 180))
        self.text(draw, (1455, 1039), '2026.11.11  /  TRIAL 04', 18, (139, 158, 180))
        self.base = np.asarray(self.base)[:, :, ::-1].copy()
        corners = np.array([[x,y,z] for x in (data.low[0],data.high[0])
                            for y in (data.low[1],data.high[1]) for z in (data.low[2],data.high[2])])
        extents = [np.ptp((corners-self.center) @ self.basis(y), axis=0)[:2] for y in (.6,.72,.84)]
        extent = np.max(extents, axis=0)
        self.scale = min(1100/extent[0], 625/extent[1])
        self.cached_key = None
        self.cached_cloud = None

    def text(self, draw, xy, s, size=20, color=WHITE, bold=False):
        key = (size, bold)
        if key not in self.fonts:
            self.fonts[key] = ImageFont.truetype(BOLD if bold else FONT, size)
        draw.text(xy, s, font=self.fonts[key], fill=color)

    @staticmethod
    def basis(yaw):
        elevation=.64
        right=np.array([math.cos(yaw),-math.sin(yaw),0.])
        up=np.array([math.sin(yaw)*math.sin(elevation),math.cos(yaw)*math.sin(elevation),math.cos(elevation)])
        return np.column_stack((right,up,np.cross(right,up)))

    def project(self, pts, yaw=.72):
        p=(pts-self.center) @ self.basis(yaw)
        return np.column_stack((637+p[:,0]*self.scale, 569-p[:,1]*self.scale)).astype(np.int32), p[:,2]

    def xy(self, pts):
        q=pts[:,:2]-self.xycenter
        return np.column_stack((1577+q[:,0]*self.xyscale, 863-q[:,1]*self.xyscale)).astype(np.int32)

    def scene(self, steady, speed, outro=False, yaw=.72):
        d=self.d
        canvas=self.base.copy()
        points, matrices=d.map_at(steady)
        key=(d.last_snapshot,round(yaw,4))
        if key!=self.cached_key:
            area=np.zeros((770,1180,3),dtype=np.uint8)
            area[:]=(44,29,18)
            uv,depth=self.project(points,yaw)
            valid=(uv[:,0]>=45)&(uv[:,0]<1225)&(uv[:,1]>=210)&(uv[:,1]<955)
            uv,depth=uv[valid],depth[valid]
            z=points[valid,2]
            colors=cv2.applyColorMap((np.clip((z-d.zrange[0])/np.ptp(d.zrange),0,1)*255).astype(np.uint8).reshape(-1,1),cv2.COLORMAP_TURBO).reshape(-1,3)
            order=np.argsort(depth);uv,colors=uv[order],colors[order]
            for dx,dy in ((0,0),(1,0),(0,1)):
                area[uv[:,1]-210+dy,uv[:,0]-45+dx]=colors
            self.cached_cloud=area[:745]
            self.cached_key=key
        canvas[210:955,45:1225]=self.cached_cloud
        if len(matrices):
            positions=matrices[:,:3,3]
            uv,_=self.project(positions,yaw)
            cv2.polylines(canvas,[uv.reshape(-1,1,2)],False,(145,220,104),2,cv2.LINE_AA)
        oi=bisect.bisect_right(d.observer_times,steady)-1
        row=d.observer[oi] if oi>=0 else None
        valid=bool(row and row['pose_valid'] and row['state']==1 and 0 <= (steady-row['sample_steady_ns'])/1e9 <=.5)
        age=(steady-row['sample_steady_ns'])/1e9 if row else None
        current=a.matrix(row['pose']) @ d.inv_base if row and row['pose_valid'] else None
        if current is not None:
            mark,_=self.project(np.array([current[:3,3],current[:3,3]+current[:3,0]*.8]),yaw)
            color=(180,240,130) if valid else (92,193,255)
            cv2.circle(canvas,tuple(mark[0]),8,(240,245,250),-1,cv2.LINE_AA)
            cv2.circle(canvas,tuple(mark[0]),14,color,2,cv2.LINE_AA)
            cv2.arrowedLine(canvas,tuple(mark[0]),tuple(mark[1]),color,3,cv2.LINE_AA,tipLength=.3)
        # Camera and online pose use the same original sampling instant; the
        # availability timeline remains the observer's actual receipt time.
        sensor=row['stamp_ns'] if row and row['pose_valid'] else d.sensor(steady)
        canvas[190:670,1277:1877]=d.camera(sensor)
        fi=np.searchsorted(d.odom_times,d.sensor(steady),side='right')
        startfi=np.searchsorted(d.odom_times,d.sensor(d.begin))
        front=d.front[startfi:fi,:3,3]
        ni=bisect.bisect_right(d.online_times,steady)
        startni=bisect.bisect_left(d.online_times,d.begin)
        track=d.online_positions[startni:ni]
        if len(front)>1:
            cv2.polylines(canvas,[self.xy(front).reshape(-1,1,2)],False,(155,125,92),2,cv2.LINE_AA)
        if len(track)>1:
            cv2.polylines(canvas,[self.xy(track).reshape(-1,1,2)],False,(180,240,130),2,cv2.LINE_AA)
            cv2.circle(canvas,tuple(self.xy(track)[0]),5,(70,210,235),-1,cv2.LINE_AA)
        if current is not None:
            mark=self.xy(np.array([current[:3,3],current[:3,3]+current[:3,0]*1.1]))
            cv2.circle(canvas,tuple(mark[0]),6,(230,245,245),-1,cv2.LINE_AA)
            cv2.arrowedLine(canvas,tuple(mark[0]),tuple(mark[1]),(180,240,130),2,cv2.LINE_AA,tipLength=.25)
        im=Image.fromarray(canvas[:,:,::-1]);draw=ImageDraw.Draw(im)
        elapsed=(steady-d.depart)/1e9
        duration=(d.returned-d.depart)/1e9
        phase='起点静止记录' if steady<d.depart else ('返回 P0 后静止观察' if steady>=d.returned else '人工移动 · 在线建图')
        if outro:phase='采集结束 · 最后记录地图'
        self.text(draw,(1365,20),f'{max(0,elapsed):05.1f} s  /  {speed:g}×',31,WHITE,True)
        self.text(draw,(1365,70),phase,20,CYAN)
        loops=row['loop_event_id'] if row else 0
        self.text(draw,(53,939),f'关键帧  {len(matrices):03d}     已接受回环事件  {loops:02d}',22,WHITE)
        state='有效位姿' if valid else '等待新鲜位姿'
        if outro:state='最后记录位姿'
        self.text(draw,(53,217),state,20,GREEN if valid or outro else AMBER)
        self.text(draw,(53,247),f'图版本 {row["graph_version"] if row else 0}  ·  位姿采样年龄 {age*1000:.0f} ms' if age is not None else '等待后端记录',17,(173,191,210))
        if row and loops and oi>=0:
            previous=next((r for r in reversed(d.observer[:oi+1]) if r['loop_event_id']<loops),None)
            if previous and steady-previous['received_steady_ns']<1_000_000_000:
                draw.rounded_rectangle((840,215,1210,258),radius=10,fill=(35,77,61))
                self.text(draw,(855,221),'回环约束已接受 · 图优化提交',19,GREEN)
        self.text(draw,(1284,954),'青灰：FAST 前端    绿色：B 在线输出',17,(171,190,209))
        progress=np.clip((steady-d.begin)/(d.end-d.begin),0,1)
        draw.rounded_rectangle((36,1007,1888,1014),radius=3,fill=(43,58,76))
        draw.rounded_rectangle((36,1007,36+max(3,int(1852*progress)),1014),radius=3,fill=CYAN)
        self.text(draw,(1283,171),'RGB 与位姿按原采样时刻关联',15,(157,176,196))
        return np.asarray(im)[:,:,::-1].copy()

    def title(self, image, end=False):
        overlay=Image.fromarray(image[:,:,::-1]).convert('RGBA')
        shade=Image.new('RGBA',(W,H),(4,10,18,185))
        overlay=Image.alpha_composite(overlay,shade)
        draw=ImageDraw.Draw(overlay)
        if not end:
            self.text(draw,(150,290),'第 4 轮  /  三融合闭环建图',58,WHITE,True)
            self.text(draw,(153,390),'FAST-LIVO2  →  关键帧 ICP 回环  →  GTSAM iSAM2',29,CYAN)
            self.text(draw,(153,460),'逐步地图 · 在线位姿 · 同步 RGB',32,WHITE)
            self.text(draw,(153,560),'人工移动与 P0 返回实验  |  2026 年 11 月 11 日',25,(175,194,214))
            self.text(draw,(153,615),'实录数据离线重绘，保持当时发布的图优化与位姿顺序',24,(175,194,214))
        else:
            self.text(draw,(150,265),'第 4 轮采集完成',55,WHITE,True)
            self.text(draw,(153,370),'110 个关键帧     56 次已接受回环事件',34,CYAN)
            self.text(draw,(153,445),'行走 151.3 秒    ·    前端估算行程 39.35 米',30,WHITE)
            self.text(draw,(153,535),'展示范围：FAST 三融合前端 + B 在线建图后端',25,(185,201,219))
            self.text(draw,(153,587),'未评估独立真值精度；本视频不代表 C1/C2 重定位验收',23,(185,201,219))
        return np.asarray(overlay.convert('RGB'))[:,:,::-1].copy()


def verify_video(path, expected, fps):
    cap=cv2.VideoCapture(str(path));decoded=0
    assert cap.isOpened()
    assert (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))==(W,H)
    while True:
        ok,frame=cap.read()
        if not ok:break
        assert frame.shape==(H,W,3)
        decoded+=1
    cap.release()
    if decoded!=expected:raise RuntimeError(f'Decode count {decoded}/{expected}')
    return {'file':str(path.resolve()),'frames':decoded,'fps':fps,'duration_s':decoded/fps,
            'resolution':[W,H],'codec':'H.264 / AVC','bytes':path.stat().st_size,'sha256':digest(path)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--preview-only',action='store_true')
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic();cv2.setNumThreads(2)
    d=Data(args.trial.resolve());r=Renderer(d)
    if args.preview_only:
        for name,fraction in [('start',.02),('middle',.52),('end',.99)]:
            im=r.scene(round(d.begin+(d.end-d.begin)*fraction),2)
            cv2.imwrite(str(args.output/(name+'.jpg')),im)
        return
    fps=20
    paths=[args.output/'trial04_mapping_rgb_1x_1080p.mp4',args.output/'trial04_meeting_2x_1080p.mp4']
    writers=[cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'avc1'),fps,(W,H)) for path in paths]
    if not all(w.isOpened() for w in writers):raise RuntimeError('H.264 encoder unavailable')
    counts=[0,0]
    def write(index,image):
        writers[index].write(image);counts[index]+=1
    intro=[r.title(r.scene(d.begin,s)) for s in (1,2)]
    for i in range(3*fps):
        for j in (0,1):write(j,intro[j])
    samples=math.ceil((d.end-d.begin)/1e9*fps)+1
    stills=[]
    for i in range(samples):
        steady=min(d.end,d.begin+round(i*1e9/fps))
        im=r.scene(steady,1);write(0,im)
        if i%2==0:
            im2=im.copy()
            # Speed label is presentation only; sensor content remains identical.
            label=Image.fromarray(im2[:,:,::-1]);dr=ImageDraw.Draw(label)
            dr.rectangle((1360,15,1890,63),fill=BG)
            r.text(dr,(1365,20),f'{max(0,(steady-d.depart)/1e9):05.1f} s  /  2×',31,WHITE,True)
            write(1,np.asarray(label)[:,:,::-1].copy())
        if i in (0,samples//3,2*samples//3,samples-1):
            name=args.output/f'frame_{i:05d}.jpg';cv2.imwrite(str(name),im);stills.append(im)
        if i%200==0:print(f'RENDER {i}/{samples} wall={time.monotonic()-started:.1f}s',flush=True)
    for i in range(4*fps):
        yaw=.72+.12*i/max(1,4*fps-1)
        for j,speed in enumerate((1,2)):write(j,r.scene(d.end,speed,True,yaw))
    for j,speed in enumerate((1,2)):
        end=r.title(r.scene(d.end,speed,True,.84),True)
        for i in range(4*fps):write(j,end)
    for writer in writers:writer.release()
    cv2.imwrite(str(args.output/'preview.jpg'),stills[2],[cv2.IMWRITE_JPEG_QUALITY,95])
    contact=np.vstack([np.hstack([cv2.resize(im,(960,540)) for im in stills[:2]]),
                       np.hstack([cv2.resize(im,(960,540)) for im in stills[2:]])])
    cv2.imwrite(str(args.output/'contact_sheet.jpg'),contact)
    result={'trial':str(args.trial.resolve()),'map_uuid':d.manifest['map_uuid'],
            'source_manifest_sha256':digest(d.capture/'map/manifest.json'),
            'renderer_sha256':digest(__file__),'source_bag_read_only':True,'hardware_or_ros_nodes_started':False,
            'map_source':'recorded /fast_livo/backend/optimized_path snapshots; past geometry redrawn on each recorded update',
            'online_pose_source':'original closure_samples.jsonl by receipt steady time; no fitted or future poses',
            'snapshot_timing':'recording timestamp mapped through preceding recorded /clock plus elapsed host time',
            'rgb_time_association':'recorded /front/image_sync header +100ms exactly once, nearest to displayed original pose stamp',
            'maximum_rgb_effective_stamp_difference_ms':max(abs(x) for x in d.camera_offsets)/1e6,
            'display_voxel_m':.085,'display_bounds_xyz':[d.low.tolist(),d.high.tolist()],
            'last_recorded_snapshot_vs_final_manifest_max_matrix_error':d.final_snapshot_error,
            'recorded_graph_snapshots':len(d.snapshots),'keyframes':len(d.frames),'accepted_loop_events':d.manifest['loop_event_id'],
            'physical_sync_proven':False,'independent_ground_truth':False,'audio':False,
            'videos':[verify_video(path,count,fps) for path,count in zip(paths,counts)],
            'wall_seconds':time.monotonic()-started}
    (args.output/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
