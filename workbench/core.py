"""ROS-free state reduction, immutable assets, presets and indexed light recordings."""
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import queue
import re
import sqlite3
import threading
import time
import uuid
import zlib

import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MAP = ROOT/'outputs/fast-livo-demo-changed-scene-20261110/navigation_map_05'
DEFAULT_ROUTE = ROOT/'config/exact_mppi/office_corridor_preparation_20261116.yaml'

def json_safe(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [json_safe(v) for v in value]
    return value

def pose(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('目标必须是 [x, y, yaw]')
    out = [float(v) for v in value]
    if not all(math.isfinite(v) for v in out): raise ValueError('坐标必须有限')
    out[2] = (out[2]+math.pi) % (2*math.pi)-math.pi
    return out

def yaw(q):
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))

def xyzpose(p):
    return [p['position']['x'], p['position']['y'], yaw(p['orientation'])]

def stamp(d):
    s = d.get('header', {}).get('stamp', {})
    return int(s.get('sec',0))*10**9+int(s.get('nanosec',0))

def atomic(path, data):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix('.tmp'); tmp.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False)); tmp.replace(path)

class Assets:
    def __init__(self, folders=None):
        import sys
        sys.path.insert(0,str(ROOT/'scripts'))
        from fast_livo2.navigation.asset import load_map
        self.maps={}
        body=yaml.safe_load((ROOT/'config/fast_livo2/navigation_prep.yaml').read_text())['body']
        length,width=body['reference_envelope_length_width_m']
        for folder in folders or [DEFAULT_MAP]:
            folder=Path(folder).resolve(); m=load_map(folder)
            key=hashlib.sha256(str(folder).encode()).hexdigest()[:16]
            cfg=yaml.safe_load((folder/'map.yaml').read_text())
            image=Image.open(folder/'map.pgm').convert('L')
            # Render exact map-server classifications, rather than treating unknown as free.
            pixels=[]
            for v in image.getdata():
                prob=v/255 if cfg.get('negate',0) else 1-v/255
                pixels.append((24,32,43) if prob>cfg['occupied_thresh'] else ((244,246,248) if prob<cfg['free_thresh'] else (152,163,176)))
            rgb=Image.new('RGB',image.size); rgb.putdata(pixels); buf=io.BytesIO(); rgb.save(buf,'PNG')
            self.maps[key]={'id':key,'name':'办公室 / 走廊' if folder==DEFAULT_MAP else folder.parent.name,
                'map_uuid':m['map_uuid'],'source_manifest_sha256':m['source_manifest_sha256'],
                'width':image.width,'height':image.height,'resolution':cfg['resolution'],'origin':cfg['origin'],
                'body':{'length':length,'width':width,'footprint':body['footprint_xy']},
                'folder':str(folder),'archive':m['source_archive'],'png':buf.getvalue()}
        if not self.maps: raise ValueError('没有可用的正式地图')
    def public(self, key):
        return {k:v for k,v in self.maps[key].items() if k not in ('png','folder','archive')}
    def get(self,key):
        if key not in self.maps: raise ValueError('未知地图')
        return self.maps[key]

class Presets:
    def __init__(self, path, assets):
        self.path=Path(path); self.assets=assets
        self.data=json.loads(self.path.read_text()) if self.path.exists() else []
        if not self.data and DEFAULT_ROUTE.exists():
            route=yaml.safe_load(DEFAULT_ROUTE.read_text())
            if any(m['map_uuid']==route['map_uuid'] and m['source_manifest_sha256']==route['source_manifest_sha256'] for m in self.assets.maps.values()):
                self.import_data(route,save=False)
    def import_data(self,data,save=True):
        if data.get('frame')!='map': raise ValueError('预设坐标系必须为 map')
        valid=[m for m in self.assets.maps.values() if m['map_uuid']==data.get('map_uuid') and m['source_manifest_sha256']==data.get('source_manifest_sha256')]
        if not valid: raise ValueError('预设地图身份不匹配')
        rows=data.get('waypoints',data.get('presets',[]))
        if not 1<=len(rows)<=128: raise ValueError('预设需要 1–128 个点')
        new=[]
        for r in rows:
            name=str(r.get('label') or r.get('name','')).strip()
            if not name or len(name)>100: raise ValueError('目标名称需要 1–100 字符')
            new.append({'id':uuid.uuid4().hex,'map_id':valid[0]['id'],'name':name,'pose':pose(r['pose']),
                        'candidate':not data.get('human_confirmed',False)})
        if len(self.data)+len(new)>512: raise ValueError('预设数量超过 512')
        self.data.extend(new)
        if save: atomic(self.path,self.data)
        return new
    def export(self,map_id):
        m=self.assets.get(map_id)
        return {'schema_version':1,'frame':'map','map_uuid':m['map_uuid'],
                'source_manifest_sha256':m['source_manifest_sha256'],'human_confirmed':False,
                'presets':[{'name':r['name'],'pose':r['pose']} for r in self.data if r['map_id']==map_id]}

class State:
    """Latest samples plus bounded body path. No heartbeat can refresh the sample stamp."""
    def __init__(self,map_id,map_uuid):
        self.map_id,self.map_uuid=map_id,map_uuid
        self.latest={}; self.received={}; self.clock_ns=0
        self.trace=[]; self.last_pose_stamp=None; self.identity=None; self.break_path=True; self.accepted_pose=None
        self.stopped_since=None;self.wheel_stamp=None;self.wheel_source=None
        cfg=yaml.safe_load((ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml').read_text())
        motion=cfg['motion']
        self.motion_limits={k:motion[k] for k in ('stop_linear','stop_angular','stop_dwell')}
        self.freshness_limits={k:float(v)*1000 for k,v in cfg['timing'].items()}
    def consume(self,topic,data,received_ns=None):
        now=received_ns or time.time_ns()
        if topic=='/clock':
            c=data['clock']; n=c['sec']*10**9+c['nanosec']
            if n<self.clock_ns: self.break_path=True; self.latest.pop('/navigation/prediction',None)
            self.clock_ns=n
        if topic=='/wheel/odom' or (topic=='/fast_livo/navigation/wheel/odom' and '/wheel/odom' not in self.latest):
            if self.wheel_source!=topic:self.stopped_since=None;self.wheel_stamp=None
            self.wheel_source=topic
            n=stamp(data)
            try:
                twist=data['twist']['twist'];v=[twist['linear']['x'],twist['linear']['y'],twist['angular']['z']]
                stopped=all(math.isfinite(x) for x in v) and math.hypot(*v[:2])<=self.motion_limits['stop_linear'] and abs(v[2])<=self.motion_limits['stop_angular']
            except (KeyError,TypeError,ValueError):stopped=False
            if not stopped:self.stopped_since=None
            elif self.stopped_since is None or self.wheel_stamp is None or not 0<=n-self.wheel_stamp<=250_000_000:self.stopped_since=n
            self.wheel_stamp=n
        if topic=='/localization/estimate':
            reason=data.get('reason',''); match=re.search(r'(?:^|;\s*)map=([^;]+)',reason)
            session=re.search(r'(?:^|;\s*)session=([^;]+)',reason)
            identity=(match.group(1) if match else None,session.group(1) if session else None)
            valid=(data.get('state')==1 and data.get('base_frame')=='base_link' and data.get('header',{}).get('frame_id')=='map' and identity[0]==self.map_uuid)
            n=stamp(data)
            if self.identity!=identity or (self.last_pose_stamp is not None and n<self.last_pose_stamp):
                self.break_path=True; self.accepted_pose=None
                self.latest.pop('/navigation/prediction',None); self.latest.pop('/plan',None); self.latest.pop('/navigation/route',None)
            self.identity=identity
            try:
                q=data['pose']['orientation'];norm=sum(float(q[k])**2 for k in ('x','y','z','w'))
                if not .99<=norm<=1.01:raise ValueError('invalid quaternion')
                p=pose(xyzpose(data['pose']))
            except (ValueError,KeyError,TypeError): valid=False; p=None
            if valid and n!=self.last_pose_stamp and p:
                if self.break_path and self.trace: self.trace.append(None)
                previous=next((r for r in reversed(self.trace) if r is not None),None)
                if previous is None or self.break_path or math.hypot(p[0]-previous[0],p[1]-previous[1])>=.015 or abs((p[2]-previous[2]+math.pi)%(2*math.pi)-math.pi)>=.02:
                    self.trace.append(p)
                self.trace=self.trace[-4000:]; self.break_path=False
            if valid:self.accepted_pose=p
            else:
                self.break_path=True; self.latest.pop('/navigation/prediction',None)
            self.last_pose_stamp=n
            data={**data,'body_pose':self.accepted_pose,'contract_valid':valid}
        if topic in ('/plan','/navigation/prediction'):
            if data.get('header',{}).get('frame_id')!='map': return
            data={**data,'points':[xyzpose(p['pose']) for p in data.get('poses',[])]}
            data.pop('poses',None)
        if topic in ('/navigation/route','/navigation/state','/navigation/execution','/base/execution_state') and isinstance(data.get('data'),str):
            try: data=json.loads(data['data'])
            except ValueError: data={'reason':'invalid_json'}
        self.latest[topic]=json_safe(data); self.received[topic]=now
    def snapshot(self,now_ns=None,clock_ns=None):
        now=now_ns or time.time_ns(); c=self.clock_ns if clock_ns is None else clock_ns
        loc=copy.deepcopy(self.latest.get('/localization/estimate'))
        if loc:
            # Original sensor clock age and independent receipt age are both visible.
            loc['sample_age_ms']=(c-stamp(loc))/1e6 if c and stamp(loc)>0 else None
            loc['receipt_age_ms']=(now-self.received['/localization/estimate'])/1e6
            loc['valid']=bool(loc.get('contract_valid') and loc['sample_age_ms'] is not None and
                0<=loc['sample_age_ms']<=self.freshness_limits['localization_sample_max_age'] and
                0<=loc['receipt_age_ms']<=self.freshness_limits['localization_receipt_max_age'])
        topics=copy.deepcopy({k:v for k,v in self.latest.items() if k not in ('/clock','/localization/estimate')})
        env=topics.get('/workbench/environment')
        if env and not 0<=c-env.get('stamp_ns',0)<=250_000_000:env.update(valid=False,reason='observation_expired')
        reference=now if self.wheel_source=='/wheel/odom' else c
        wheel_age=(reference-self.wheel_stamp)/1e6 if self.wheel_stamp else None
        stopped=bool(self.stopped_since is not None and self.wheel_stamp and self.wheel_stamp-self.stopped_since>=self.motion_limits['stop_dwell']*1e9 and 0<=reference-self.wheel_stamp<=250_000_000 and 0<=now-self.received.get(self.wheel_source,0)<=250_000_000)
        return {'map_id':self.map_id,'feedback_stopped':stopped,'feedback_source':self.wheel_source,'feedback_sample_age_ms':wheel_age,'motion_limits':self.motion_limits,'freshness_limits':self.freshness_limits,'map_uuid':self.map_uuid,'clock_ns':c,'localization':loc,
                'trace':copy.deepcopy(self.trace),'topics':topics,
                'topic_receipt_age_ms':{k:(now-v)/1e6 for k,v in self.received.items()},'server_time_ns':now}

class Journal:
    """Bounded asynchronous disk writer. Recording faults never hold the ROS thread."""
    def __init__(self,folder,metadata,min_free_bytes=200_000_000):
        self.folder=Path(folder); self.folder.mkdir(parents=True,exist_ok=False)
        self.metadata={**metadata,'schema_version':1,'kind':'ranger_workbench_light','started_ns':time.time_ns()}
        atomic(self.folder/'manifest.json',self.metadata)
        self.q=queue.Queue(32); self.error=None; self.dropped=0; self.min_free_bytes=min_free_bytes
        self.thread=threading.Thread(target=self._write,daemon=True); self.thread.start()
    def add(self,kind,data,at_ns=None):
        if self.error: return
        try: self.q.put_nowait((at_ns or time.time_ns(),kind,json.dumps(data,ensure_ascii=False,allow_nan=False)))
        except queue.Full: self.dropped+=1
    def _write(self):
        import shutil
        db=None
        try:
            next_disk=0;segment=0;segment_start=None
            while True:
                row=self.q.get()
                if row is None:break
                if time.monotonic()>next_disk:
                    if shutil.disk_usage(self.folder).free<self.min_free_bytes:raise OSError('disk_low_recording_stopped')
                    next_disk=time.monotonic()+2
                if db is None or row[0]-segment_start>=600_000_000_000:
                    if db:db.commit();db.close()
                    name='events.sqlite3' if segment==0 else f'events-{segment:04}.sqlite3'
                    db=sqlite3.connect(self.folder/name);segment+=1;segment_start=row[0]
                    db.execute('create table events(seq integer primary key, t integer, kind text, data blob)')
                    db.execute('create index events_time on events(t,seq)')
                # Compression belongs to the disk thread, never the ROS callbacks.
                db.execute('insert into events(t,kind,data) values(?,?,?)',(row[0],row[1],zlib.compress(row[2].encode(),3)))
                if self.q.empty():db.commit()
        except Exception as exc:self.error=str(exc)
        finally:
            if db:db.commit();db.close()
    def close(self):
        if self.thread.is_alive():
            self.q.put(None,timeout=3); self.thread.join(5)
        atomic(self.folder/'result.json',{'ended_ns':time.time_ns(),'error':self.error,'dropped':self.dropped,'finalized':not self.thread.is_alive()})

class Replay:
    def __init__(self,folder):
        self.folder=Path(folder); self.metadata=json.loads((self.folder/'manifest.json').read_text())
        self.files=sorted(self.folder.glob('events*.sqlite3'));self.bounds=[]
        for f in self.files:
            with self.connection(f) as db:a,b=db.execute('select min(t),max(t) from events').fetchone()
            if a is not None:self.bounds.append((a,b,f))
        if not self.bounds:raise ValueError('记录为空')
        self.bounds.sort();self.start=min(a for a,_,_ in self.bounds);self.end=max(b for _,b,_ in self.bounds)
    def connection(self,path):return sqlite3.connect(f'file:{path}?mode=ro',uri=True)
    @staticmethod
    def decode(value):return json.loads(zlib.decompress(value) if isinstance(value,bytes) else value)
    def seek(self,seconds):
        sec=float(seconds)
        if not math.isfinite(sec):raise ValueError('回放时间必须有限')
        t=min(self.end,max(self.start,self.start+int(sec*1e9)))
        snapshots=[];events=[]
        for a,_,f in reversed(self.bounds):
            if a>t:continue
            with self.connection(f) as db:
                if not snapshots:
                    row=db.execute("select t,data from events where kind='snapshot' and t<=? order by t desc,seq desc limit 1",(t,)).fetchone()
                    if row:snapshots.append(row)
                if len(events)<30:
                    events.extend(db.execute("select t,data from events where kind='event' and t<=? order by t desc,seq desc limit ?",(t,30-len(events))).fetchall())
            if snapshots and len(events)>=30:break
        snap=self.decode(snapshots[0][1]) if snapshots else None
        # Age the latest snapshot to the requested instant, without inventing poses.
        if snap and snap.get('localization'):
            loc=snap['localization'];delta=max(0,t-snapshots[0][0])/1e6
            for key in ('sample_age_ms','receipt_age_ms'):
                if loc.get(key) is not None:loc[key]+=delta
            limits=snap.get('freshness_limits',{'localization_sample_max_age':250,'localization_receipt_max_age':250})
            if (loc.get('sample_age_ms') is None or not 0<=loc['sample_age_ms']<=limits['localization_sample_max_age'] or
                    loc.get('receipt_age_ms') is None or not 0<=loc['receipt_age_ms']<=limits['localization_receipt_max_age']):loc['valid']=False
            if delta>250:snap['feedback_stopped']=False
        return {'view':'replay','time_s':(t-self.start)/1e9,'duration_s':(self.end-self.start)/1e9,
                'snapshot':snap,'events':[{'time_ns':r[0],**self.decode(r[1])} for r in reversed(events)],'metadata':self.metadata}
