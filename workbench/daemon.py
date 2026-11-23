"""Robot-side owner: survives browsers and HTTP restarts; starts hardware only on session.start."""
import argparse
import asyncio
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from workbench.core import Assets, Journal, ROOT, State, atomic, pose, stamp
from scripts.fast_livo2.camera_startup import camera_startup_detail, log_tail
sys.path.insert(0,str(ROOT/'scripts'))
from session_manager import read_session, session_live, identity, same_process

TERMINAL={'SUCCEEDED','FAILED','CANCELED'}

def prepare_chassis_can():
    """Only called by an explicit session start; reuse the reviewed fixed command."""
    from scripts.fast_livo2.wheel_capture import check_other_drivers, can_state, prepare_can
    check_other_drivers()
    return prepare_can(can_state())

def owned_startup(directory, record):
    run,state=read_session(directory)
    if not state or not record:
        return None, None
    guardian = state.get('guardian', {})
    if any(guardian.get(key) != record.get(key) for key in ('pid', 'start_ticks', 'boot_id')):
        return None, None
    return run, state


def startup_failure(name,directory,record):
    """Keep the actual nested launch error, rather than only its exit code."""
    run, state = owned_startup(directory, record)
    if not state:
        return None
    if name == 'localization' and 'camera(STARTING' in state.get('reason', ''):
        detail = camera_startup_detail(run, state.get('components', {}))
        if detail:
            return f'定位启动超时。{detail}'
    errors=[]
    for entry in state.get('components',{}):
        lines = log_tail(run / (entry + '.log')).splitlines()
        errors.extend(line for line in lines if '[ERROR]' in line or 'ERROR:' in line or 'RuntimeError:' in line)
    return f'{name} 启动失败：'+(errors[-1][-1200:] if errors else state.get('reason','未知启动错误'))

def chassis_control_feedback(state,now_ns=None):
    """Remote presence is not required; the original CAN feedback is authoritative."""
    now=time.time_ns() if now_ns is None else now_ns
    system=state.latest.get('/system_state')
    if not system or not 0<=now-state.received.get('/system_state',0)<=250_000_000:
        return False,'等待新鲜底盘系统反馈'
    if not 0<=now-stamp(system)<=250_000_000:
        return False,'底盘系统反馈原采样时间已过期'
    if system.get('vehicle_state')!=0 or system.get('error_code')!=0:
        return False,f"底盘异常：vehicle_state={system.get('vehicle_state')}，error_code={system.get('error_code')}"
    mode=system.get('control_mode')
    if mode==1:return True,'底盘已反馈 CAN 控制模式'
    if mode==3:return False,'遥控器当前占用控制权（control_mode=3），电脑请求尚未获准；遥控器可保持关闭，开启时需切到 CAN 档'
    return False,f'等待底盘确认电脑 CAN 控制请求（control_mode={mode}）'

def localization_progress(reason,metrics):
    labels={
        'searching_waiting_query_window':'收集查询点云',
        'searching_geometric_registration':'查询已齐，正在检索和几何匹配',
        'verifying_wait_disjoint_query_scans':'等待下一组独立点云',
        'verifying_stationary_geometry':'累计静止强几何确认',
        'verifying_stationary_support_restarted':'已丢弃较弱初始候选，重新累计静止确认',
        'verifying_wait_strong_stationary_geometry':'当前匹配未达静止强几何要求，等待更好观测',
        'verifying_independent_motion_support':'当前候选需进一步几何或运动确认',
        'verifying_insufficient_time_or_motion_baseline':'等待满足确认时间及位姿一致性',
    }
    detail=labels.get(reason,reason)
    if reason in labels:detail+=f'（{reason}）'
    count,required=metrics.get('query_window_frames'),metrics.get('query_window_required_frames')
    if count is not None:detail+=f'，查询窗口 {count}/{required} 帧'
    count,elapsed=metrics.get('support_queries'),metrics.get('support_elapsed_s')
    if count is not None:detail+=f'，确认 {count} 组（需至少 3 组强几何、跨度 1 秒）' if metrics.get('support_anchor_strong') else f'，候选支持 {count} 组'
    if isinstance(elapsed,(float,int)):detail+=f'，跨度 {elapsed:.1f} 秒'
    if reason=='verifying_wait_strong_stationary_geometry':
        rmse,limit=metrics.get('best_rmse_m'),metrics.get('stationary_max_rmse_m')
        if isinstance(rmse,(float,int)) and isinstance(limit,(float,int)):detail+=f'，匹配 RMSE {rmse:.3f}m／静止上限 {limit:.3f}m'
    return detail

class Owner:
    def __init__(self,assets,data,demo=False,bridge=None):
        self.assets=assets; self.data=Path(data); self.data.mkdir(parents=True,exist_ok=True)
        self.demo=demo; self.bridge=bridge
        self.map_id=next(iter(assets.maps)); self.state=State(self.map_id,assets.get(self.map_id)['map_uuid'])
        self.session={'id':None,'state':'STOPPED','reason':'尚未启动设备'}
        self.task=None; self.children={};self.journal=None; self.events=[];self.keys={}
        self.start_job=None;self.stop_job=None;self.op_lock=asyncio.Lock();self.raw=None;self.jobs=set();self.alive=True
        self.reset_job=None;self.navigation_reset={'state':'IDLE'}
    def emit(self,kind,**data):
        event={'kind':kind,'time_ns':time.time_ns(),**data};self.events.append(event);self.events=self.events[-80:]
        if self.journal:self.journal.add('event',event)
    def consume(self,topic,value): self.state.consume(topic,value)
    def result(self,task_id,status):
        if self.task and self.task['id']==task_id:
            self.task['state']=status
            if status=='FAILED':
                navigation=self.state.latest.get('/navigation/state',{})
                self.task['reason']=navigation.get('failure_cause') or navigation.get('reason') or 'navigation_failed'
                self.task['terminal_reason']=navigation.get('reason')
                self.task['planning_failure']=copy.deepcopy(navigation.get('planning_failure'))
            if self.bridge:self.bridge.arm(False)
            self.emit('task_result',task_id=task_id,status=status,reason=self.task.get('reason'))
    def snapshot(self):
        snap=self.state.snapshot()
        return {**snap,'view':'live','demo':self.demo,'session':copy.deepcopy(self.session),
                'task':copy.deepcopy(self.task),'navigation_reset':copy.deepcopy(self.navigation_reset),'events':copy.deepcopy(self.events[-30:]),
                'recording':{'light':str(self.journal.folder) if self.journal else None,
                    'error':self.journal.error if self.journal else None,'dropped':self.journal.dropped if self.journal else 0,
                    'raw':self.raw_status()},'device_control_available':not self.demo}
    def raw_status(self):
        if not self.raw:return None
        p,folder=self.raw
        try:status=json.loads((folder/'recording.json').read_text())
        except (OSError,ValueError):status={'state':'STARTING'}
        return {'folder':str(folder),'running':p.poll() is None,**status}
    def background(self,coro):
        task=asyncio.create_task(coro);self.jobs.add(task);task.add_done_callback(self.jobs.discard)
        return task
    def check_session(self,req):
        if req.get('session_id')!=self.session['id'] or not self.session['id']:raise ValueError('会话已变化，请刷新状态')
    async def request(self,method,req):
        if method=='snapshot': return self.snapshot()
        if method not in ('session.start','session.stop','goal','cancel','navigation.reset','raw.start','raw.stop'):
            raise ValueError('未知操作')
        if req.get('view')!='live': raise ValueError('回放只读，不能操作设备')
        if self.demo: raise ValueError('演示模式不操作设备或导航目标')
        key=req.get('request_id')
        if not isinstance(key,str) or not 8<=len(key)<=128:raise ValueError('缺少幂等请求标识')
        fingerprint=json.dumps([method,req],sort_keys=True)
        async with self.op_lock:
            if key in self.keys:
                old_fingerprint,value=self.keys[key]
                if old_fingerprint!=fingerprint:raise ValueError('请求标识被用于不同操作')
                return value
            if method=='session.start':
                if self.session['state'] not in ('STOPPED','FAILED') or any(p.poll() is None for p,_,_ in self.children.values()):raise ValueError('已有会话，不能重复启动')
                m=self.assets.get(req['map_id'])
                if req.get('profile','standard') not in ('standard','narrow'):raise ValueError('未知净空配置')
                self.map_id=m['id'];self.state=State(m['id'],m['map_uuid'])
                self.children={};self.task=None;self.navigation_reset={'state':'IDLE'}
                self.session={'id':uuid.uuid4().hex,'state':'STARTING','map_id':m['id'],
                    'profile':req.get('profile','standard'),'reason':'正在启动底盘、三融合定位与导航'}
                self.start_job=self.background(self.start())
                value=copy.deepcopy(self.session)
            elif method=='session.stop':
                self.check_session(req);self.session['state']='STOPPING';self.stop_job=self.background(self.stop_after_start())
                value=copy.deepcopy(self.session)
            elif method=='goal':
                self.check_session(req)
                if self.navigation_reset['state']=='RESETTING':raise ValueError('规划控制正在重置，请稍候')
                if self.session['state']!='READY':raise ValueError('设备会话尚未就绪')
                snap=self.state.snapshot()
                if not (snap.get('localization') or {}).get('valid'):raise ValueError('定位无效或已过期')
                if not self.state.latest.get('/navigation_ready',{}).get('data'):raise ValueError('导航输入尚未就绪')
                for t in ('/navigation_ready','/system_state','/base/execution_state'):
                    if snap['topic_receipt_age_ms'].get(t,1e9)>250:raise ValueError('底盘或导航反馈过期: '+t)
                control_ready,control_reason=chassis_control_feedback(self.state)
                if not control_ready:raise ValueError(control_reason)
                if req['map_id']!=self.map_id:raise ValueError('目标地图不匹配')
                target=pose(req['pose'])
                navigation=self.state.latest.get('/navigation/state',{})
                if not self.task and navigation.get('state') not in (None,'IDLE','SUCCEEDED','CANCELED','FAILED'):
                    raise ValueError('导航器存在外部任务，工作台不接管')
                active=self.task and self.task['state'] not in TERMINAL
                if active and not req.get('replace'):raise ValueError('已有目标，请使用替换目标')
                if active:
                    self.bridge.arm(False); await self.bridge.cancel(self.task['id'])
                task={'id':uuid.uuid4().hex,'session_id':self.session['id'],'pose':target,'state':'ACCEPTING'}
                self.task=task
                self.heartbeat()
                self.state.latest.pop('/plan',None);self.state.latest.pop('/navigation/route',None);self.state.latest.pop('/navigation/prediction',None)
                try:
                    # False then true belongs only to this explicit execute operation.
                    self.bridge.arm(False)
                    await self.bridge.goal(task['id'],target)
                    if self.task is task and task['state']=='ACCEPTING':
                        task['state']='RUNNING';self.bridge.arm(True)
                except Exception as exc:
                    self.bridge.arm(False);task.update(state='FAILED',reason=str(exc));raise
                self.emit('goal',task_id=task['id'],pose=target);value=copy.deepcopy(task)
            elif method=='cancel':
                self.check_session(req)
                if not self.task or req.get('task_id')!=self.task['id']:raise ValueError('任务已变化')
                self.bridge.arm(False)
                if self.task['state'] not in TERMINAL:
                    self.task['state']='CANCELING';await self.bridge.cancel(self.task['id'])
                self.emit('cancel',task_id=self.task['id']);value=copy.deepcopy(self.task)
            elif method=='navigation.reset':
                self.check_session(req)
                if self.session['state']!='READY':raise ValueError('导航会话尚未启动')
                if self.navigation_reset['state']=='RESETTING':raise ValueError('规划控制正在重置')
                nav=self.state.latest.get('/navigation/state',{})
                active=self.task and self.task['state'] not in TERMINAL
                retry=self.navigation_reset['state']=='FAILED' and nav.get('state')=='RESETTING'
                if not active and not retry and nav.get('state') not in (None,'IDLE','SUCCEEDED','CANCELED','FAILED'):
                    raise ValueError('存在外部或未确认的导航任务，不能重置')
                self.bridge.arm(False)
                task_id=self.task['id'] if self.task else None
                if active:self.task['state']='CANCELING'
                self.navigation_reset={'state':'RESETTING','reason':'正在结束旧目标并重置规划控制'}
                self.emit('navigation_reset_requested',task_id=task_id)
                self.reset_job=self.background(self.reset_navigation(task_id))
                value=copy.deepcopy(self.navigation_reset)
            elif method=='raw.start':
                self.check_session(req)
                if self.session['state']!='READY':raise ValueError('会话尚未就绪')
                if self.raw and self.raw[0].poll() is None:raise ValueError('已在录制')
                duration=float(req.get('seconds',900))
                if not 1<=duration<=3600:raise ValueError('录制时长必须为 1–3600 秒')
                folder=ROOT/'bags'/('workbench-raw-'+uuid.uuid4().hex[:12])
                manifest={'session_id':self.session['id'],'map_id':self.map_id,**self.assets.public(self.map_id)}
                # Reuse the online source/config snapshots. Original inputs remain byte-identical.
                manifest['localization_folder']=self.localization_folder
                folder.mkdir();atomic(folder/'workbench_manifest.json',manifest)
                log=(folder/'worker.log').open('xb')
                p=subprocess.Popen([sys.executable,'-m','workbench.raw_record','--folder',str(folder),'--seconds',str(duration)],stdout=log,stderr=subprocess.STDOUT)
                log.close();self.raw=(p,folder);value=self.raw_status()
            else:
                self.check_session(req);await self.stop_raw();value=self.raw_status()
            self.keys[key]=(fingerprint,value)
            if len(self.keys)>2048:self.keys.pop(next(iter(self.keys)))
            return value
    async def reset_navigation(self,task_id):
        try:
            ack=await self.bridge.reset(task_id)
            self.navigation_reset['reason']='旧目标已结束，正在初始化规划控制；定位继续运行'
            end=time.monotonic()+35.
            while time.monotonic()<end:
                if self.session['state']!='READY':raise asyncio.CancelledError()
                nav=self.state.latest.get('/navigation/state',{});worker=nav.get('worker') or {}
                fresh=0<=time.time_ns()-self.state.received.get('/navigation/state',0)<=250_000_000
                if fresh and nav.get('state')=='FAILED' and worker.get('error'):
                    raise RuntimeError('规划控制重置失败：'+str(worker['error']))
                if (fresh and nav.get('state')=='IDLE' and nav.get('generation',-1)>=ack['generation'] and
                        worker.get('ready') and worker.get('nonce') and worker['nonce']!=ack['previous_worker_nonce']):
                    self.task=None
                    for topic in ('/plan','/navigation/route','/navigation/prediction'):self.state.latest.pop(topic,None)
                    self.navigation_reset={'state':'READY','reason':'旧任务已清除，规划控制已就绪；请重新执行目标'}
                    self.emit('navigation_reset_ready');return
                await asyncio.sleep(.05)
            raise TimeoutError('规划控制重置等待超时，运动使能保持关闭')
        except asyncio.CancelledError:
            self.navigation_reset={'state':'CANCELED','reason':'会话停止，规划控制重置已取消'}
        except Exception as exc:
            self.navigation_reset={'state':'FAILED','reason':str(exc)};self.emit('navigation_reset_failed',reason=str(exc))
        finally:self.bridge.arm(False)
    def spawn(self,name,argv,directory,env=None):
        if name in self.children and self.children[name][0].poll() is None:raise RuntimeError('已有自有进程: '+name)
        _,state=read_session(directory)
        if session_live(state):raise RuntimeError('外部会话占用: '+str(directory))
        logdir=self.data/'logs'/self.session['id'];logdir.mkdir(parents=True,exist_ok=True)
        log=(logdir/(name+'.log')).open('xb')
        p=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,env=env)
        log.close();self.children[name]=(p,identity(p.pid),Path(directory))
    async def wait_ready(self,name,timeout):
        p,record,folder=self.children[name];end=time.monotonic()+timeout
        while time.monotonic()<end:
            if self.session['state']!='STARTING':raise RuntimeError('启动已取消')
            if p.poll() is not None:
                raise RuntimeError(startup_failure(name,folder,record) or name+' 启动退出 '+str(p.returncode))
            run,s=owned_startup(folder, record)
            if s:
                self.session['progress']={'component':name,'state':s.get('state'),
                    'components':{k:v.get('state') for k,v in s.get('components',{}).items()}}
                if s.get('state')=='RUNNING':return
                if name == 'localization':
                    detail = camera_startup_detail(run, s.get('components', {}))
                    self.session['reason'] = detail or '正在启动传感器、三融合定位与公共接口'
            await asyncio.sleep(.2)
        raise TimeoutError(name+' 就绪超时')
    async def stop_after_start(self):
        if self.start_job and not self.start_job.done():await self.start_job
        await self.stop()
    async def wait_chassis_control(self,timeout=10.):
        end=time.monotonic()+timeout;reason='等待底盘 CAN 控制反馈'
        while time.monotonic()<end:
            if self.session['state']!='STARTING':raise RuntimeError('启动已取消')
            ready,reason=chassis_control_feedback(self.state)
            self.session.update(stage='chassis_control',reason=reason)
            if ready:return
            await asyncio.sleep(.1)
        raise RuntimeError(reason+'；未使能导航，不自动反复抢占控制权')
    async def wait_localization_ready(self,timeout=120.):
        end=time.monotonic()+timeout;reason='等待重定位结果'
        while time.monotonic()<end:
            if self.session['state']!='STARTING':raise RuntimeError('启动已取消')
            snap=self.state.snapshot();localization=snap.get('localization') or {}
            if (localization.get('valid') and self.state.latest.get('/navigation_ready',{}).get('data') and
                    snap['topic_receipt_age_ms'].get('/navigation_ready',1e9)<=250):return
            path=Path(self.localization_folder)/'localization_health.json'
            try:
                health=json.loads(path.read_text());backend=health.get('backend') or {};metrics=health.get('backend_metrics') or {}
                reason=backend.get('reason') or localization.get('reason') or reason
                detail=localization_progress(reason,metrics)
                self.session['progress']={'component':'relocalization','state':'WAITING',
                    'components':{'relocalization':detail}}
                self.session['reason']='等待重定位确认：'+detail
            except (OSError,ValueError):self.session['reason']='等待重定位确认：'+reason
            self.session['stage']='relocalization'
            # A dead managed frontend must be reported immediately, not as a C2 timeout.
            for name in ('base','localization'):
                child=self.children.get(name)
                if child and child[0].poll() is not None:
                    raise RuntimeError(startup_failure(name,child[2],child[1]) or name+' 进程退出')
            await asyncio.sleep(.2)
        raise RuntimeError('重定位未就绪：'+self.session.get('reason',reason)+'；导航未启动、未使能运动')
    async def start(self):
        try:
            if self.session['state']!='STARTING':return
            # Check all fixed session slots before starting any device.
            for folder in [ROOT/'.runtime/sessions/base',ROOT/'.runtime/sessions/navigation',ROOT/'.runtime/sessions/fast_livo_online_localization']:
                if session_live(read_session(folder)[1]):raise RuntimeError('已有外部会话，仅观察: '+str(folder))
            m=self.assets.get(self.map_id)
            (self.data/'records').mkdir(exist_ok=True)
            self.journal=Journal(self.data/'records'/('live-'+self.session['id']),self.assets.public(self.map_id))
            self.emit('session_start',session_id=self.session['id'])
            env=dict(os.environ,RANGER_RUNTIME_CONFIG=str(ROOT/'config/profiles/fast_livo2_frontend.yaml'))
            self.session.update(stage='can',reason='正在检查 USB-CAN，必要时启用 can0 / 500kbit/s')
            can=await asyncio.to_thread(prepare_chassis_can)
            logdir=self.data/'logs'/self.session['id'];logdir.mkdir(parents=True,exist_ok=True)
            atomic(logdir/'can_preparation.json',can)
            self.emit('can_ready',enabled=can['enabled_by_capture'])
            if self.session['state']!='STARTING':raise RuntimeError('启动已取消')
            self.session.update(stage='base',reason='CAN 已就绪，正在启动底盘并等待反馈')
            self.spawn('base',[sys.executable,str(ROOT/'scripts/robot_session.py'),'base','--execute','--control-authorized'],ROOT/'.runtime/sessions/base',env)
            await self.wait_ready('base',60)
            # The driver already sends EnableCommandedMode at explicit startup.
            # Do not ask for an RC transmitter or repeatedly overwrite RC takeover.
            await self.wait_chassis_control()
            folder=ROOT/'bags'/('workbench-localization-'+self.session['id'][:12]);self.localization_folder=str(folder)
            self.session.update(stage='localization',reason='底盘反馈已就绪，正在启动传感器、三融合定位与公共接口')
            self.spawn('localization',['bash',str(ROOT/'scripts/localize_fast_livo2.sh'),'--execute','--continuous','--record-navigation',
                '--archive',m['archive'],'--public-tf','--navigation-map',m['folder'],'--visualization','none',
                '--output',str(folder),'--domain',os.environ.get('ROS_DOMAIN_ID','215')],ROOT/'.runtime/sessions/fast_livo_online_localization',env)
            await self.wait_ready('localization',110)
            try:
                identity=json.loads((folder/'source_identity.json').read_text())
                sources=identity['definitions']['frontend_sources_sha256']
                self.session['frontend']={'optimized':bool(sources.get('latency_patch_sha256')),
                    'binary_sha256':sources['binary_sha256'],'source_sha256':identity['frontend_sources_sha256']}
            except (OSError,ValueError,KeyError):self.session['frontend']=None
            await self.wait_localization_ready()
            self.session.update(stage='navigation',reason='定位已确认，正在启动导航与优化器')
            self.spawn('navigation',[sys.executable,str(ROOT/'scripts/robot_session.py'),'navigation','--execute','--control-authorized'],
                ROOT/'.runtime/sessions/navigation',dict(env,RANGER_NAVIGATION_MAP=str(Path(m['folder'])/'map.yaml'),RANGER_CLEARANCE_PROFILE=self.session['profile']))
            await self.wait_ready('navigation',70)
            self.bridge.arm(False);self.session.update(state='READY',stage='ready',progress=None,reason='设备已启动，等待有效定位后可执行目标')
            if (self.session.get('frontend') or {}).get('optimized'):
                self.session['reason']='设备已启动；低延迟三融合前端已加载，等待有效定位后可执行目标'
            self.emit('session_ready')
        except Exception as exc:
            self.session.update(state='FAILED',reason=str(exc));self.emit('session_failure',reason=str(exc))
            await self.stop(failed=True)
    async def stop_raw(self):
        if self.raw and self.raw[0].poll() is None:
            self.raw[0].send_signal(signal.SIGINT)
            end=time.monotonic()+20
            while self.raw[0].poll() is None and time.monotonic()<end:await asyncio.sleep(.1)
            if self.raw[0].poll() is None:self.emit('recording_stop_timeout')
    async def stop(self,failed=False):
        if self.reset_job and not self.reset_job.done():
            self.reset_job.cancel()
            try:await self.reset_job
            except asyncio.CancelledError:self.navigation_reset={'state':'CANCELED','reason':'会话停止，规划控制重置已取消'}
        if self.bridge and (self.children or self.task):
            self.bridge.arm(False)
            if self.task and self.task['state'] not in TERMINAL:
                try:await self.bridge.cancel(self.task['id'])
                except Exception as exc:self.emit('cancel_error',reason=str(exc))
                self.task['state']='FAILED' if failed else 'CANCELED'
        await self.stop_raw()
        for name in ('navigation','localization','base'):
            row=self.children.get(name)
            if not row:continue
            p,record,_=row
            if p.poll() is None and same_process(record):p.send_signal(signal.SIGINT)
            end=time.monotonic()+50
            while p.poll() is None and time.monotonic()<end:await asyncio.sleep(.1)
            if p.poll() is None:
                self.session.update(state='FAILED',reason='受管停止超时: '+name);return
        self.emit('session_stop')
        self.children={}
        if self.journal:
            await asyncio.to_thread(self.journal.close);self.journal=None
        if not failed:self.session.update(state='STOPPED',reason='自有会话已停止')
    def heartbeat(self):
        atomic(self.data/'owner_heartbeat.json',{'identity':identity(os.getpid()),'steady':time.monotonic(),
            'active':bool(self.task and self.task['state'] not in TERMINAL)})
    async def tick(self):
        last_record=0; last_heartbeat=0
        while self.alive:
            if self.bridge:self.bridge.spin()
            if time.monotonic()-last_heartbeat>.1:
                self.heartbeat()
                last_heartbeat=time.monotonic()
            if self.session['state']=='READY':
                dead=[n for n,(p,_,_) in self.children.items() if p.poll() is not None]
                if dead:
                    self.session.update(state='FAILED',reason='自有进程退出: '+','.join(dead));self.background(self.stop(failed=True))
            if self.journal and time.monotonic()-last_record>.2:
                self.journal.add('snapshot',self.snapshot());last_record=time.monotonic()
            await asyncio.sleep(.01)

async def serve(args):
    assets=Assets(args.maps);owner=Owner(assets,args.data,args.demo)
    if not args.demo:
        from workbench.ros_bridge import RosBridge
        owner.bridge=RosBridge(owner.consume,owner.result)
    socket=Path(args.socket);socket.parent.mkdir(parents=True,exist_ok=True)
    if socket.exists():socket.unlink()
    async def handle(reader,writer):
        try:
            line=await asyncio.wait_for(reader.readline(),3)
            request=json.loads(line)
            result=await owner.request(request['method'],request.get('data',{}))
            out={'ok':True,'result':result}
        except Exception as exc:out={'ok':False,'error':str(exc)}
        writer.write((json.dumps(out,ensure_ascii=False,allow_nan=False)+'\n').encode())
        await writer.drain();writer.close();await writer.wait_closed()
    server=await asyncio.start_unix_server(handle,path=socket,limit=1_000_000);socket.chmod(0o600)
    tick=asyncio.create_task(owner.tick());stop=asyncio.Event()
    loop=asyncio.get_running_loop()
    for sig in (signal.SIGINT,signal.SIGTERM):loop.add_signal_handler(sig,stop.set)
    try:await stop.wait()
    finally:
        owner.session['state']='STOPPING'
        if owner.start_job and not owner.start_job.done():await owner.start_job
        if owner.stop_job and not owner.stop_job.done():await owner.stop_job
        await owner.stop();owner.alive=False;await tick
        if owner.bridge:owner.bridge.close()
        server.close();await server.wait_closed();socket.unlink(missing_ok=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--socket',required=True);p.add_argument('--data',required=True);p.add_argument('--demo',action='store_true');p.add_argument('--map',dest='maps',action='append')
    asyncio.run(serve(p.parse_args()))
