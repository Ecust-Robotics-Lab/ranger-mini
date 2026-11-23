import {MapView} from './map.js';
const $=s=>document.querySelector(s),text=(s,v)=>{$(s).textContent=v??'—';};
let maps=[],presets=[],live=null,view='live',ws,replayId=null,replay=null,playing=false,lastPlay=0,seekBusy=false,seekQueued=null,lastMessage=0;
const modes={0:'行驶 DRIVE',1:'平移 PARALLEL',2:'旋转 SPIN'},states={STOPPED:'未启动',STARTING:'启动中',READY:'已启动',STOPPING:'停止中',FAILED:'失败',RUNNING:'执行中',ACCEPTING:'接受目标',CANCELING:'取消中',CANCELED:'已取消',SUCCEEDED:'已完成'};
Object.assign(states,{IDLE:'等待新目标',RESETTING:'规划控制重置中'});
const reasons={permission_off:'运动未使能',localization_invalid:'定位暂不可用',feedback_settling:'等待反馈停稳',steering_alignment:'等待轮角对齐',command_watchdog:'指令看门狗停车',guidance_unobserved:'路径前段观测不足',unobserved_sweep:'预测扫掠区域未观测',no_progress_timeout:'持续无进展，任务失败',inputs_invalid:'输入暂时失效',waiting_for_inputs:'等待输入',no_route:'未找到可行路线',feedback_settling:'等待反馈稳定',no_progress:'未取得有效进展',waiting_inputs:'等待输入',tracking:'正常跟踪',zero_command:'零速保持',mode_switch:'切换运动模式',mode_switch_timeout:'模式切换超时',valid:'检查通过'};
Object.assign(reasons,{start_static_clearance_insufficient:'起点车体可容纳，但静态规划额外净空不足；可用窄通道配置复核',goal_static_clearance_insufficient:'目标车体的静态规划净空不足',start_static_collision_or_unknown:'起点完整车体与静态障碍或未知区域重叠',goal_static_collision_or_unknown:'目标完整车体与静态障碍或未知区域重叠',start_live_obstacle_collision:'起点车体安全范围内存在实时障碍',goal_live_obstacle_collision:'目标车体安全范围内存在实时障碍，请重新选点',start_unobserved:'起点车体观测覆盖不足',goal_unobserved:'目标车体观测覆盖不足'});
Object.assign(reasons,{planning_control_reset:'规划控制已就绪，请重新执行目标'});
Object.assign(reasons,{coverage_body_unobserved:'保护范围未知格超过容许数量',coverage_edge_budget_exceeded:'边缘未知格超过容许数量',coverage_known_obstacle:'观测范围内有已知障碍格',coverage_outside_grid:'车体超出局部观测网格'});
function explain(r){if(!r)return '等待输入';return reasons[r]?`${reasons[r]} · ${r}`:r;}
function toast(message){text('#toast',message);$('#toast').hidden=false;clearTimeout(toast.timer);toast.timer=setTimeout(()=>$('#toast').hidden=true,5500);}
async function api(path,body){const r=await fetch('/api/v1/'+path,{method:body===undefined?'GET':'POST',headers:body===undefined?{}:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body)});if(r.status===401){$('#login').hidden=false;throw Error('请使用工控机访问密钥登录');}const value=await r.json().catch(()=>({error:r.statusText}));if(!r.ok)throw Error(value.error||r.statusText);return value;}
const map=new MapView($('#map'),p=>{if(view==='live'){setGoal(p);clearPreview();}});
function goal(){const values=['#goalX','#goalY','#goalYaw'].map(id=>$(id).value);if(values.some(v=>!v.trim()||!Number.isFinite(Number(v))))throw Error('请填写完整、有限的目标坐标和朝向');return [Number(values[0]),Number(values[1]),Number(values[2])*Math.PI/180];}
function setGoal(p){$('#goalX').value=p[0].toFixed(3);$('#goalY').value=p[1].toFixed(3);$('#goalYaw').value=(p[2]*180/Math.PI).toFixed(1);map.goal=p;map.draw();}
function clearPreview(){map.preview=null;text('#previewResult','执行时会从当前有效定位重新规划。');map.draw();}
function updatePresets(){const id=$('#mapSelect').value;map.presets=presets.filter(p=>p.map_id===id);$('#presetSelect').replaceChildren(new Option('选择预设目标…',''),...map.presets.map(p=>new Option((p.candidate?'候选 · ':'')+p.name,p.id)));map.draw();}
async function selectMap(id){$('#mapSelect').value=id;const m=maps.find(m=>m.id===id);if(!m)return;await map.setMap(m);text('#mapTitle',m.name);text('#mapIdentity',`UUID ${m.map_uuid.slice(0,12)}…`);updatePresets();clearPreview();}
function speed(v){return Array.isArray(v)&&v.length===3&&v.every(Number.isFinite)?Math.hypot(v[0],v[1]).toFixed(3):'—';}
function render(s){if(!s)return;map.setSnapshot(s);const t=s.topics||{},n=t['/navigation/state']||{},loc=s.localization,execution=t['/navigation/execution']||{},base=t['/base/execution_state']||{},system=t['/system_state']||{},task=s.task;
 const locLabel=loc?(loc.valid?'有效':loc.state===2?'几何匹配暂退化':!loc.contract_valid?'定位未就绪':loc.sample_age_ms<0?'等待时钟对齐':loc.receipt_age_ms>250?'定位更新断流':'定位采样过期'):'未记录 / 无输入';
 text('#localization',locLabel);text('#readyBadge',loc?.valid?'定位有效':locLabel);$('#readyBadge').className='badge '+(loc?.valid?'good':'bad');
 const problem=task?.planning_failure||n.planning_failure;let taskReason=explain(task?.reason||n.failure_cause||n.reason||loc?.reason);
 if(problem?.details?.start?.reason&&problem?.details?.goal?.reason)taskReason+='；'+explain(problem.details.goal.reason);
 if((n.failure_cause||n.reason)==='guidance_unobserved'&&n.coverage_diagnostic){const d=n.coverage_diagnostic.blocked||n.coverage_diagnostic.current;if(d)taskReason+=`；${reasons[d.reason]||d.reason}，保护范围未知 ${d.unknown_protected_cells}/${d.protected_unknown_cell_budget??0} 格，边缘未知 ${d.unknown_edge_cells}/${d.unknown_cell_budget} 格，总外扩 ${Math.round(d.total_margin_m*1000)}mm`;}
 text('#taskState',states[task?.state]||states[n.state]||n.state||'尚无导航任务');text('#taskReason',taskReason);
 text('#requestedMode',modes[execution.mode??n.segment_mode??t['/navigation/control']?.mode]||n.mode);text('#actualMode',modes[t['/motion_state']?.motion_mode??t['/navigation/actual_mode']?.data]);text('#controlOwner',t['/navigation/control_owner']?.data||execution.owner);
 const receipt=s.topic_receipt_age_ms||{};text('#chassisControl',receipt['/system_state']<=250?({0:'待机，等待电脑请求',1:'CAN / 电脑控制',3:'遥控器优先'})[system.control_mode]||'未知控制模式':'等待新鲜反馈');text('#permitted',t['/ranger/motion_permitted']?.data&&receipt['/ranger/motion_permitted']<=250?'已放行':'未放行 / 无新鲜反馈');
 const wheelTopic=t['/wheel/odom']?'/wheel/odom':'/fast_livo/navigation/wheel/odom';
 const wheel=t[wheelTopic]?.twist?.twist;const stopLimits=s.motion_limits||{stop_linear:.015,stop_angular:.025,stop_dwell:.5};
 const measured=wheel?[wheel.linear.x,wheel.linear.y,wheel.angular.z]:execution.measured_body;
 const freshWheel=receipt[wheelTopic]<=250&&(s.feedback_sample_age_ms==null||(s.feedback_sample_age_ms>=0&&s.feedback_sample_age_ms<=250));
 text('#stationary',freshWheel&&Array.isArray(measured)&&measured.every(Number.isFinite)?(s.feedback_stopped?'反馈已停稳':Math.hypot(measured[0],measured[1])<=stopLimits.stop_linear&&Math.abs(measured[2])<=stopLimits.stop_angular?'等待连续停稳':'反馈仍有运动'):'等待新鲜反馈');
 text('#requestedSpeed',speed(execution.requested_body||base.requested_body||(t['/navigation/control']?.twist?[t['/navigation/control'].twist.linear.x,t['/navigation/control'].twist.linear.y,t['/navigation/control'].twist.angular.z]:null)));text('#measuredSpeed',freshWheel?speed(measured):'—');
 text('#controlSpeedLimit',Number.isFinite(n.control_limits?.max_speed)?`${n.control_limits.max_speed.toFixed(2)} m/s · ${n.control_limits.max_yaw_rate.toFixed(2)} rad/s`:'等待导航参数');
 const dist=t['/navigation/minimum_distance']?.data;text('#minimumDistance',Number.isFinite(dist)?dist.toFixed(2)+' m':'—');
 const age=x=>Number.isFinite(x)?Math.round(x)+' ms':'—';text('#sampleAge',age(loc?.sample_age_ms));text('#receiptAge',age(loc?.receipt_age_ms));$('#sampleAge').title=`定位采样年龄上限 ${s.freshness_limits?.localization_sample_max_age??250} ms`;
 text('#inputHealth',n.inputs_valid===true?'输入有效':n.input_reason||'等待输入');text('#computeTime',n.metrics?.end_to_end_s!=null?(n.metrics.end_to_end_s*1000).toFixed(1)+' ms':'—');
 const p=loc?.body_pose;text('#poseText',p?`x ${p[0].toFixed(2)}   y ${p[1].toFixed(2)}   θ ${(p[2]*180/Math.PI).toFixed(1)}°`:'等待有效机体位姿');
 text('#mapMode',view==='replay'?'历史回放 · 只读':loc?.valid?'实时 · 定位有效':'实时 · 无有效定位');
 $('#events').replaceChildren(...(s.events?.length?s.events.slice(-15).reverse().map(e=>{const li=document.createElement('li'),time=document.createElement('time');time.textContent=new Date(e.time_ns/1e6).toLocaleTimeString();li.append(time,document.createTextNode(`${e.kind} ${e.reason||e.status||''}`));return li;}):[Object.assign(document.createElement('li'),{textContent:view==='replay'?'该记录未保存工作台任务事件':'暂无任务事件'})]));
 if(view==='replay'){text('#sessionBadge','历史记录');text('#sessionReason','回放已封存数据，不操作当前设备。');text('#recordingBadge','历史数据');text('#recordingStatus','缺少的历史话题显示为空，不补造数据。');for(const b of document.querySelectorAll('[data-live]'))b.disabled=true;return;}
 text('#sessionBadge',states[s.session?.state]||'未启动');text('#sessionReason',s.session?.reason||'');
 $('#sessionBadge').className='badge '+(s.session?.state==='FAILED'?'bad':s.session?.state==='STARTING'?'starting':s.session?.state==='READY'?'good':'');
 const failure=s.session?.state==='FAILED';$('#sessionFailure').hidden=!failure;text('#sessionFailure',failure?'启动 / 会话失败：'+s.session.reason:'');
 const resetting=s.navigation_reset?.state==='RESETTING'||(n.state==='RESETTING'&&!['FAILED','CANCELED'].includes(s.navigation_reset?.state));
 if(resetting||s.navigation_reset?.state==='FAILED'){text('#taskState',resetting?'规划控制重置中':'规划控制重置失败');text('#taskReason',s.navigation_reset?.reason||'等待规划控制就绪');}
 const progress=s.session?.progress?.components;const componentNames={probe:'输入检查',base:'底盘',lidar:'雷达',camera:'相机',writer:'时间同步',frontend:'三融合前端',localizer:'定位',relocalization:'重定位',bridge:'公共接口',navigation:'导航'};
 text('#sessionProgress',progress?Object.entries(progress).map(([k,v])=>`${componentNames[k]||k}：${({READY:'就绪',STARTING:'启动中',WAITING:'等待',EXITED:'退出'})[v]||v}`).join(' · '):'');
 if(failure){text('#taskState','设备会话失败');text('#taskReason',s.session.reason);}
 const disabled=s.demo||!s.device_control_available;for(const b of document.querySelectorAll('[data-live]'))b.disabled=disabled;
 $('#startSession').disabled=disabled||!['STOPPED','FAILED'].includes(s.session?.state);
 $('#stopSession').disabled=disabled||['STOPPED','STOPPING'].includes(s.session?.state);
 $('#mapSelect').disabled=!['STOPPED','FAILED'].includes(s.session?.state);$('#profile').disabled=$('#mapSelect').disabled;
 $('#execute').disabled=disabled||resetting||n.worker?.ready===false||s.session?.state!=='READY'||!loc?.valid||receipt['/system_state']>250||system.control_mode!==1||system.vehicle_state!==0||system.error_code!==0;
 $('#resetNavigation').disabled=disabled||resetting||s.session?.state!=='READY';
 const active=task&&!['SUCCEEDED','FAILED','CANCELED'].includes(task.state);$('#execute').firstChild.textContent=active?'替换当前目标 ':'执行单点导航 ';
 $('#cancel').disabled=disabled||!active;$('#rawRecord').disabled=disabled||s.session?.state!=='READY';
 const recording=s.recording||{};text('#recordingBadge',recording.error?'记录异常':recording.light?'轻量记录中':'待会话启动');
 text('#recordingStatus',recording.error|| (recording.raw?.running?(recording.raw.state==='RECORDING'?'原始传感器录制中，结束后导航继续。':'录包器准备中，等待必要话题写入确认。'):recording.light?'轻量记录中；原始传感器包未录制。':'默认轻量记录；原始传感器包按需开启。'));
 text('#rawRecord',recording.raw?.running?'停止录包':'开始录包');text('#footerStatus',s.demo?'演示模式：使用真实预建图，未启动任何设备。':'任务由工控机持有，浏览器断开后继续当前目标。');
}
function connect(){ws?.close();const socket=new WebSocket(`${location.protocol==='https:'?'wss:':'ws:'}//${location.host}/api/v1/events`);ws=socket;socket.onmessage=e=>{lastMessage=Date.now();live=JSON.parse(e.data);text('#connection','工控机已连接');$('#connectionDot').className='dot online';$('#demoBadge').hidden=!live.demo;if(view==='live')render(live);updateBackground();};socket.onclose=()=>{if(socket!==ws)return;text('#connection','连接中断 · 任务不受影响');$('#connectionDot').className='dot';setTimeout(connect,2000);};}
function updateBackground(){$('#backgroundTask').hidden=!(view==='replay'&&live?.task&&!['SUCCEEDED','FAILED','CANCELED'].includes(live.task.state));}
function liveRequest(data={}){return {view,session_id:live?.session?.id,request_id:crypto.randomUUID?crypto.randomUUID():Array.from(crypto.getRandomValues(new Uint8Array(16)),v=>v.toString(16).padStart(2,'0')).join(''),...data};}
async function operate(path,data={}){if(view!=='live')throw Error('回放页面只读');return api(path,liveRequest(data));}
async function waitJob(id){for(let i=0;i<2400;i++){const v=await api('jobs/'+id);if(v.state==='DONE')return v.result;if(v.state==='FAILED')throw Error(v.reason);await new Promise(r=>setTimeout(r,150));}throw Error('后台任务等待超时');}
async function refreshRecordings(){const rows=await api('recordings');$('#recordSelect').replaceChildren(...rows.map(r=>new Option((r.kind==='rosbag'?'原包 · ':'回放 · ')+r.name,r.id)));if(!rows.length)text('#replayStatus','暂无已封存记录。会话运行时会自动保存轻量数据。');}
async function switchView(next){view=next;map.mode=next;for(const id of ['#goalX','#goalY','#goalYaw','#presetSelect','#presetName','#savePreset','#importPresets','#preview'])$(id).disabled=next==='replay';playing=false;text('#play','播放');$('#liveTab').classList.toggle('active',view==='live');$('#replayTab').classList.toggle('active',view==='replay');$('#replayControls').hidden=view!=='replay';updateBackground();if(view==='live'){await selectMap(live?.map_id||$('#mapSelect').value);render(live);}else{for(const b of document.querySelectorAll('[data-live]'))b.disabled=true;await refreshRecordings();map.preview=null;map.setSnapshot(null);text('#mapMode','历史回放 · 选择记录');}}
async function seek(t){if(!replayId)return;if(seekBusy){seekQueued=t;return;}seekBusy=true;try{const result=await api(`replays/${encodeURIComponent(replayId)}?time=${t}`);if(view!=='replay')return;replay=result;$('#seek').max=Math.max(.1,result.duration_s);$('#seek').value=result.time_s;text('#replayTime',`${clock(result.time_s)} / ${clock(result.duration_s)}`);if(result.snapshot){if(result.snapshot.map_id!==$('#mapSelect').value)await selectMap(result.snapshot.map_id);render({...result.snapshot,events:result.events.length?result.events:result.snapshot.events});}text('#replayStatus','历史记录 · 定位估计轨迹 · 回放操作不会发布 ROS 消息。');}finally{seekBusy=false;if(seekQueued!==null){const value=seekQueued;seekQueued=null;seek(value).catch(e=>toast(e.message));}}}
function clock(s){return `${Math.floor(s/60)}:${String(Math.floor(s%60)).padStart(2,'0')}`;}
function bind(selector,fn){$(selector).addEventListener('click',async e=>{try{await fn(e);}catch(e){toast(e.message);}});}
$('#loginForm').addEventListener('submit',async e=>{e.preventDefault();try{await api('login',{key:$('#accessKey').value});$('#accessKey').value='';$('#login').hidden=true;await init();}catch(e){text('#loginError',e.message);}});
bind('#fit',()=>map.fit());bind('#zoomIn',()=>map.zoom(1.25));bind('#zoomOut',()=>map.zoom(.8));bind('#follow',()=>{map.follow=!map.follow;$('#follow').classList.toggle('active',map.follow);map.setSnapshot(view==='live'?live:replay?.snapshot);});
bind('#liveTab',()=>switchView('live'));bind('#replayTab',()=>switchView('replay'));
$('#mapSelect').addEventListener('change',()=>selectMap($('#mapSelect').value).catch(e=>toast(e.message)));
$('#profile').addEventListener('change',clearPreview);
for(const s of ['#goalX','#goalY','#goalYaw'])$(s).addEventListener('change',()=>{try{map.goal=goal();clearPreview();}catch(e){toast(e.message);}});
$('#presetSelect').addEventListener('change',()=>{const p=presets.find(p=>p.id===$('#presetSelect').value);if(p){setGoal(p.pose);clearPreview();}});
for(const input of document.querySelectorAll('[data-layer]'))input.addEventListener('change',()=>{map.layers[input.dataset.layer]=input.checked;map.draw();});
bind('#savePreset',async()=>{await api('presets',{map_id:$('#mapSelect').value,name:$('#presetName').value,pose:goal()});presets=await api('presets');updatePresets();toast('目标已保存');});
bind('#exportPresets',async()=>{const data=await api('presets/export?map_id='+$('#mapSelect').value),url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download='ranger-presets.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
$('#importPresets').addEventListener('change',async e=>{try{const file=e.target.files[0];if(file.size>250000)throw Error('文件过大');await api('presets/import',JSON.parse(await file.text()));presets=await api('presets');updatePresets();toast('预设已导入');}catch(e){toast(e.message);}e.target.value='';});
bind('#preview',async()=>{$('#preview').disabled=true;text('#previewResult','静态搜索中，最长约 5 秒…');try{const start=view==='live'&&live?.localization?.valid?live.localization.body_pose:[Number($('#startX').value),Number($('#startY').value),Number($('#startYaw').value)*Math.PI/180];const request=await api('previews',{map_id:$('#mapSelect').value,start,goal:goal(),profile:$('#profile').value,mode:live?.topics['/motion_state']?.motion_mode||0});const result=await waitJob(request.job_id);map.preview=result;map.draw();text('#previewResult',result.accepted?`静态扫掠通过 · ${result.length_m.toFixed(2)} m · ${result.segments.length} 段。实时可执行性仍需检查。`:`未找到静态路线：${result.reason}`);}finally{$('#preview').disabled=false;}});
bind('#startSession',async()=>{$('#startSession').disabled=true;try{const session=await operate('session/start',{map_id:$('#mapSelect').value,profile:$('#profile').value});text('#sessionReason',session.reason);text('#sessionBadge','启动中');toast('启动请求已受理，可在运行会话中查看每一步状态');}catch(e){$('#startSession').disabled=false;throw e;}});
bind('#stopSession',async()=>{await operate('session/stop');toast('正在取消任务并停止自有会话');});
bind('#execute',async()=>{await operate('goals',{map_id:$('#mapSelect').value,pose:goal(),replace:!!live?.task&&!['SUCCEEDED','FAILED','CANCELED'].includes(live.task.state)});toast('目标已由工控机接受');});
bind('#cancel',async()=>{await operate('goals/cancel',{task_id:live?.task?.id});toast('已撤销使能并请求取消；请查看反馈停稳状态');});
bind('#resetNavigation',async()=>{$('#resetNavigation').disabled=true;try{await operate('navigation/reset');toast('正在重置规划控制；请等待就绪后重新执行目标');}catch(e){$('#resetNavigation').disabled=false;throw e;}});
bind('#rawRecord',async()=>{await operate(live?.recording?.raw?.running?'recordings/stop':'recordings/start',{seconds:Number($('#recordSeconds').value)});toast('录制请求已受理');});
bind('#loadRecord',async()=>{playing=false;let id=$('#recordSelect').value;if(!id)throw Error('请选择记录');if(id.startsWith('bag:')){text('#replayStatus','正在离线索引原包，不启动设备或向 ROS 发布…');const job=await api('replays/import',{id});const result=await waitJob(job.job_id);id=result.output.split('/').pop();await refreshRecordings();$('#recordSelect').value=id;}replayId=id;await seek(0);});
bind('#play',()=>{if(!replayId)throw Error('请先加载记录');playing=!playing;lastPlay=Date.now();text('#play',playing?'暂停':'播放');});
$('#seek').addEventListener('input',()=>{playing=false;text('#play','播放');seek(Number($('#seek').value)).catch(e=>toast(e.message));});
setInterval(()=>{if(playing&&replay&&view==='replay'){const now=Date.now(),t=Math.min(replay.duration_s,replay.time_s+(now-lastPlay)/1000*Number($('#speed').value));lastPlay=now;seek(t).catch(e=>toast(e.message));if(t>=replay.duration_s){playing=false;text('#play','播放');}}if(lastMessage&&Date.now()-lastMessage>1000&&view==='live'){text('#connection','数据中断 · 任务状态待重连');$('#connectionDot').className='dot';if(live?.localization?.valid){const stale=structuredClone(live);stale.localization.valid=false;render(stale);}$('#execute').disabled=true;}},150);
async function init(){maps=await api('maps');presets=await api('presets');$('#mapSelect').replaceChildren(...maps.map(m=>new Option(m.name,m.id)));await selectMap(maps[0].id);setGoal(goal());live=await api('snapshot');if(live.task?.pose)setGoal(live.task.pose);render(live);connect();}
api('maps').then(async()=>{$('#login').hidden=true;await init();}).catch(()=>{});
