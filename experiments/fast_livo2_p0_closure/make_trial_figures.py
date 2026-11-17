#!/usr/bin/env python3
"""Three source-traceable meeting figures, no trajectory fitting or ROS nodes."""
import argparse,csv,hashlib,json,math,sys
from collections import Counter
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle,ConnectionPatch
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import analysis as a
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=FONT).get_name(),
 'font.size':12,'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False,
 'axes.labelcolor':'#334155','text.color':'#172b43','xtick.color':'#475569','ytick.color':'#475569',
 'axes.edgecolor':'#b4bfca','savefig.facecolor':'white','pdf.fonttype':3,'svg.fonttype':'path'})
COLORS={'frontend':'#2463a6','optimized_history':'#c64b3e','online_backend':'#21876f'}
NAMES={'frontend':'FAST 前端','optimized_history':'最终历史轨迹','online_backend':'在线 B 后端'}
ORANGE='#ed8b23'

def load(p):return json.loads(Path(p).read_text())
def save_json(p,v):Path(p).write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def pcd(path):
 with Path(path).open('rb') as f:
  while f.readline().strip()!=b'DATA binary':pass
  return np.frombuffer(f.read(),dtype='<f4').reshape(-1,4)[:,:3].copy()
def tidy(ax):ax.grid(True,color='#e5ebf0',linewidth=.6,alpha=.8);ax.set_axisbelow(True)
def export(fig,out,name):
 for ext in ('png','svg','pdf'):fig.savefig(out/(name+'.'+ext),dpi=240)
 plt.close(fig)

def factors(trial,out):
 m=load(trial/'capture/map/manifest.json');assert load(trial/'capture/map/COMMITTED')['manifest_sha256']==sha(trial/'capture/map/manifest.json')
 frames=m['keyframes'];X=np.array([k['X_map_imu']['xyz'] for k in frames]);U=np.array([k['U_local_imu']['xyz'] for k in frames])
 path=np.r_[0,np.cumsum(np.linalg.norm(np.diff(U,axis=0),axis=1))]
 loops=[f for f in m['factors'] if f['loop']];odom=[f for f in m['factors'] if not f['loop'] and not f['prior']]
 assert len(frames)==86 and len(loops)==44 and len(odom)==85
 edges=lambda fs:np.array([[X[f['i'],:2],X[f['j'],:2]] for f in fs])
 lengths=np.array([path[f['j']]-path[f['i']] for f in loops]);times=np.array([(frames[f['j']]['stamp_ns']-frames[f['i']]['stamp_ns'])*1e-9 for f in loops])
 endpoints=np.concatenate(edges(loops));density=np.sum(np.linalg.norm(endpoints[:,None]-endpoints[None,:],axis=2)<.65,axis=1)
 center=endpoints[np.argmax(density)];lo=center-[.85,.70];hi=center+[.85,.70]
 accepted=[r for r in a.jsonl(trial/'capture/map/candidates.jsonl') if r.get('decision')=='accepted_and_optimized']
 overlap=[]
 for p,q in zip(accepted,accepted[1:]):
  aa,bb=set(p['target_scan_ids']),set(q['target_scan_ids']);overlap.append(len(aa&bb)/min(len(aa),len(bb)))
 repeated=sum(v>=.5 for v in overlap)
 fig=plt.figure(figsize=(16,9));gs=fig.add_gridspec(2,2,width_ratios=[1.65,1],left=.065,right=.96,bottom=.20,top=.84,wspace=.21,hspace=.44)
 main=fig.add_subplot(gs[0,0]);reuse=fig.add_subplot(gs[1,0]);zoom=fig.add_subplot(gs[0,1]);hist=fig.add_subplot(gs[1,1])
 def graph(ax,zoomed=False):
  ax.add_collection(LineCollection(edges(odom),colors='#a6adb5',linewidths=1.3,zorder=1))
  ax.add_collection(LineCollection(edges(loops),colors=ORANGE,linewidths=1.6 if zoomed else 1.3,alpha=.78,zorder=2))
  ax.scatter(X[:,0],X[:,1],s=21 if zoomed else 15,color='#294763',zorder=3)
  ax.scatter(*X[0,:2],marker='*',s=190,color='#1d9d7b',edgecolor='white',zorder=6)
  ax.scatter(*X[-1,:2],marker='X',s=100,color='#b54b65',edgecolor='white',zorder=6)
  ax.set_aspect('equal',adjustable='box');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)');tidy(ax)
 graph(main);graph(zoom,True)
 main.set_xlim(X[:,0].min()-.7,X[:,0].max()+.7);main.set_ylim(X[:,1].min()-.7,X[:,1].max()+.7)
 zoom.set_xlim(lo[0],hi[0]);zoom.set_ylim(lo[1],hi[1]);zoom.set_title('B  回环密集区域放大',loc='left',fontsize=14,pad=12)
 main.set_title('A  全部关键帧与约束（采用最终优化位姿）',loc='left',fontsize=14,pad=14)
 main.add_patch(Rectangle(lo,*(hi-lo),fill=False,edgecolor=ORANGE,linewidth=1.6,linestyle='--'))
 main.annotate('起点 KF 0',X[0,:2],xytext=(20,36),textcoords='offset points',arrowprops={'arrowstyle':'-','color':'#1d9d7b'},color='#1d8a6b')
 main.annotate('终点 KF 85',X[-1,:2],xytext=(-8,-42),textcoords='offset points',arrowprops={'arrowstyle':'-','color':'#b54b65'},color='#a63e59')
 used=Counter(f['i'] for f in loops)
 for label_index,(k,count) in enumerate(used.most_common(2)):
  if np.all(X[k,:2]>lo)&np.all(X[k,:2]<hi):zoom.annotate(f'KF {k}（{count} 条）',X[k,:2],xytext=((-35,32) if label_index==0 else (15,15)),textcoords='offset points',fontsize=10,arrowprops={'arrowstyle':'-','color':'#8696a5'})
 legend=[Line2D([0],[0],marker='o',color='none',markerfacecolor='#294763',label='关键帧'),Line2D([0],[0],color='#a6adb5',label='相邻 FAST 里程计因子'),Line2D([0],[0],color=ORANGE,label='已接受回环因子')]
 main.legend(handles=legend,loc='lower right',frameon=False,fontsize=11)
 reuse.scatter([f['j'] for f in loops],[f['i'] for f in loops],s=38,color=ORANGE,edgecolor='white',linewidth=.5)
 reuse.set_title('D  历史关键帧复用：每点对应一条回环因子',loc='left',fontsize=14,pad=12)
 reuse.set_xlabel('当前查询关键帧 ID（j）');reuse.set_ylabel('历史候选关键帧 ID（i）');tidy(reuse)
 reuse.text(.03,.95,'同一横向行中的多个点 = 多次连接相同历史关键帧',transform=reuse.transAxes,va='top',fontsize=10,color='#64748b')
 hist.hist(lengths,bins=np.arange(5,36,5),color=ORANGE,alpha=.85,edgecolor='white');tidy(hist)
 hist.set_xlabel('回环两端沿原始 FAST 关键帧链的累计距离 (m)');hist.set_ylabel('回环因子数')
 hist.set_title('C  回环跨越的轨迹段',loc='left',fontsize=14,pad=12)
 hist.text(.03,.94,f'范围 {lengths.min():.2f}–{lengths.max():.2f} m\n中位数 {np.median(lengths):.2f} m',transform=hist.transAxes,va='top',fontsize=11,bbox={'facecolor':'white','alpha':.9,'edgecolor':'none'})
 fig.text(.055,.945,'第 5 轮：回环连接集中在哪里？',fontsize=23,weight='bold')
 fig.text(.055,.895,f'86 个关键帧  ·  85 条相邻里程计因子  ·  44 条回环因子 / 44 次接受事件',fontsize=13,color='#5c6e80')
 fig.text(.055,.09,f'连续接受事件中，目标扫描集合重叠 ≥50%：{repeated}/{len(overlap)} 对；这是目标复用证据，不是统计独立性判断。',fontsize=12)
 fig.text(.055,.045,'说明：边画在最终图上；链长由原始 FAST 关键帧位置计算。边密集不能单独证明有害，须结合配准质量与图优化变化。',fontsize=11,color='#64748b')
 export(fig,out,'01_trial05_factor_graph')
 result={'keyframes':len(frames),'odom_factors':len(odom),'loop_factors':len(loops),'loop_events':m['loop_event_id'],
  'chain_distance_m':{'min':float(lengths.min()),'median':float(np.median(lengths)),'max':float(lengths.max())},
  'time_separation_s':{'min':float(times.min()),'median':float(np.median(times)),'max':float(times.max())},
  'consecutive_target_set_overlap_at_least_half':repeated,'consecutive_pairs':len(overlap),
  'zoom_bounds_xy':[lo.tolist(),hi.tolist()],'historical_target_counts':dict(used)}
 save_json(out/'01_factor_statistics.json',result)
 with (out/'01_loop_edges.csv').open('w') as f:
  writer=csv.writer(f);writer.writerow(['factor_id','i','j','raw_FAST_chain_distance_m','sample_time_separation_s'])
  for edge,length,seconds in zip(loops,lengths,times):writer.writerow([edge['id'],edge['i'],edge['j'],length,seconds])
 return result

def closure(trial,out):
 directory=Path(load(trial/'latest_analysis.json')['directory']);m=load(directory/'metrics.json');e=load(trial/'events.json')
 origin=np.array(m['trajectories']['frontend']['start']['position_xyz'][:2])
 fig=plt.figure(figsize=(16,9));gs=fig.add_gridspec(2,2,width_ratios=[1.1,1],height_ratios=[1.15,1],left=.065,right=.95,bottom=.15,top=.83,wspace=.23,hspace=.38)
 route=fig.add_subplot(gs[0,0]);note=fig.add_subplot(gs[1,0]);local=fig.add_subplot(gs[:,1])
 rows={};mids={}
 for key,color in COLORS.items():
  rows[key]=np.genfromtxt(directory/(key+'.csv'),delimiter=',',names=True)
  r=rows[key];mask=(r['sample_steady_ns']>=e['start_window'][0])&(r['sample_steady_ns']<=e['end_hold_complete_ns'])
  xy=np.column_stack((r['x_base_m'][mask],r['y_base_m'][mask]))-origin
  route.plot(xy[:,0],xy[:,1],color=color,lw=1.7,alpha=.9,label=NAMES[key])
  metrics=m['trajectories'][key]
  mids[key]=[]
  for w,style in [('start','o'),('settled','^')]:
   window=metrics['start'] if w=='start' else metrics['end']['settled']
   lo,hi=window['bounds_steady_ns'];mask=(r['sample_steady_ns']>=lo)&(r['sample_steady_ns']<=hi)
   if key=='online_backend':mask &=r['received_steady_ns']<=hi
   pts=(np.column_stack((r['x_base_m'][mask],r['y_base_m'][mask]))-origin)*100
   local.scatter(pts[:,0],pts[:,1],s=31,marker=style,color=color,alpha=.42,edgecolors='none')
   median=(np.array(window['position_xyz'][:2])-origin)*100;mids[key].append(median)
   local.scatter(*median,s=120,marker=style,color=color,edgecolor='white',linewidth=1.2,zorder=6)
  start,end=mids[key];local.annotate('',end,xytext=start,arrowprops={'arrowstyle':'->','color':color,'lw':2.1},zorder=5)
  assert abs(np.linalg.norm(end-start)/100-metrics['metrics']['settled']['position_error_m'])<1e-10
 route.set_title('A  全程三轨迹叠加',loc='left',fontsize=14,pad=12);route.set_aspect('equal',adjustable='datalim');route.set_xlabel('共同 X (m)');route.set_ylabel('共同 Y (m)');tidy(route)
 route.legend(loc='upper left',fontsize=10,frameon=False)
 route.scatter(0,0,marker='*',s=140,color='#263a51',zorder=9);route.annotate('P0 起终点附近',(0,0),xytext=(15,-35),textcoords='offset points',fontsize=10,arrowprops={'arrowstyle':'-','color':'#263a51'})
 local.set_title('B  起终窗口分布与返回闭合差',loc='left',fontsize=14,pad=12);local.set_aspect('equal',adjustable='datalim');tidy(local)
 local.set_xlabel('相对同一前端起点中位数的 X (cm)');local.set_ylabel('相对同一前端起点中位数的 Y (cm)')
 handles=[Line2D([0],[0],color=c,lw=2,label=NAMES[k]) for k,c in COLORS.items()]
 handles +=[Line2D([0],[0],marker='o',color='#6c7b8c',linestyle='none',label='起点 3 秒窗口'),Line2D([0],[0],marker='^',color='#6c7b8c',linestyle='none',label='返回后 11–13 秒窗口')]
 local.legend(handles=handles,loc='upper right',frameon=True,framealpha=.9,fontsize=10)
 note.axis('off');table=[]
 for key in COLORS:
  mm=m['trajectories'][key]['metrics']['settled'];table.append([NAMES[key],f'{mm["position_error_m"]*100:.3f}',f'{mm["yaw_error_deg"]:.3f}'])
 t=note.table(cellText=table,colLabels=['轨迹','XY 闭合差 (cm)','航向返回差 (°)'],cellLoc='center',bbox=[0,.33,1,.62]);t.auto_set_font_size(False);t.set_fontsize(12)
 for (row,col),cell in t.get_celld().items():
  cell.set_edgecolor('#e1e7ee');cell.set_facecolor('#eef3f8' if row==0 else 'white')
  if row==0:cell.set_text_props(weight='bold')
  elif col==0:cell.set_text_props(color=list(COLORS.values())[row-1])
 startspan=m['trajectories']['frontend']['start']['position_span_m']*100;endspan=m['trajectories']['frontend']['end']['settled']['position_span_m']*100
 note.text(0,.2,f'前端窗口位置跨度：起点 {startspan:.2f} cm；终点 {endspan:.2f} cm。',fontsize=12)
 note.text(0,.05,'小点：原始窗口采样；大点：分量中位数；箭头：起点 → 终点。',fontsize=11,color='#64748b')
 fig.text(.055,.945,'第 5 轮：返回闭合差与起终窗口分布',fontsize=23,weight='bold')
 fig.text(.055,.895,'人工 P0 复位 · 无独立真值 · 第 5 轮起点静止检查未通过（PARTIAL）',fontsize=13,color='#a66224')
 fig.text(.055,.085,'三条轨迹只应用同一个 XY 平移，未分别旋转、拟合或缩放；最终历史轨迹使用最终图，在线轨迹保留当时首次有效输出。',fontsize=11)
 fig.text(.055,.042,'“返回闭合差”是起终窗口中位数的 XY 距离，不是 APE/RPE。窗口波动与人工复位误差限制了对后端收益的解释。',fontsize=11,color='#64748b')
 export(fig,out,'03_trial05_return_closure')
 result={'metrics_source':str(directory/'metrics.json'),'common_origin_frontend_start_xy':origin.tolist(),
  'alignment':'one shared XY translation only; no per-trajectory fitting',
  'status':m['status'],'frontend_start_span_cm':startspan,'frontend_end_span_cm':endspan,
  'settled_metrics':{k:v['metrics']['settled'] for k,v in m['trajectories'].items()}}
 save_json(out/'03_closure_statistics.json',result);return result

def registration(folder,out):
 s=load(folder/'stages.json');assert s['success'] and s['c1_confirmation']['valid']
 assert s['map_uuid']=='160f8572a77a61856cf87e43514be769'
 target=pcd(folder/'target.pcd');sources=[pcd(folder/f'query_{name}.pcd') for name in ('before','kiss','gicp')]
 assert len(set(len(p) for p in sources))==1
 selected=s['selected_candidate'];vals=s['independent_validation'];stages=['before','kiss','gicp']
 # One common orthographic view for all three clouds, followed by the same XY
 # enlargement. No panel-specific framing, decimation or artificial transform.
 yaw=.52;elev=.54
 right=np.array([math.cos(yaw),-math.sin(yaw),0]);up=np.array([math.sin(yaw)*math.sin(elev),math.cos(yaw)*math.sin(elev),math.cos(elev)])
 project=lambda p:np.column_stack((p@right,p@up))
 projected=[project(target)]+[project(p) for p in sources];joined=np.concatenate(projected)
 lo,hi=joined.min(0),joined.max(0);pad=np.maximum((hi-lo)*.035,.15);lo-=pad;hi+=pad
 # Center the detail on a recorded fine-aligned source point nearest to its
 # own median, a fixed geometric rule independent of residual improvement.
 fine=sources[-1];center=fine[np.argmin(np.linalg.norm(fine[:,:2]-np.median(fine[:,:2],axis=0),axis=1)),:2]
 roi=np.array([center-[1.1,1.1],center+[1.1,1.1]])
 fig=plt.figure(figsize=(17,10));gs=fig.add_gridspec(2,3,left=.045,right=.97,bottom=.235,top=.82,hspace=.30,wspace=.16,height_ratios=[1.3,1])
 titles=['配准前：未施加变换','KISS-Matcher 后','small_gicp 后']
 for col,(cloud,stage,title) in enumerate(zip(sources,stages,titles)):
  ax=fig.add_subplot(gs[0,col]);z=project(cloud)
  ax.scatter(projected[0][:,0],projected[0][:,1],s=.7,color='#adb4bb',alpha=.48,rasterized=True)
  ax.scatter(z[:,0],z[:,1],s=1.4,color='#cc3571',alpha=.77,rasterized=True)
  ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_aspect('equal');ax.set_title(title,fontsize=15,pad=10)
  ax.set_xlabel('共同投影视图 U (m)');
  if col==0:ax.set_ylabel('共同投影视图 V (m)')
  tidy(ax)
  detail=fig.add_subplot(gs[1,col]);detail.scatter(target[:,0],target[:,1],s=5,color='#adb4bb',alpha=.55,rasterized=True)
  detail.scatter(cloud[:,0],cloud[:,1],s=10,color='#cc3571',alpha=.78,rasterized=True)
  detail.set_xlim(roi[0,0],roi[1,0]);detail.set_ylim(roi[0,1],roi[1,1]);detail.set_aspect('equal');tidy(detail)
  detail.set_xlabel('相同局部区域 X (m)')
  if col==0:detail.set_ylabel('Y (m)')
  v=vals[stage];elapsed='未运行求解' if stage=='before' else f'本次离线调用：{selected["coarse" if stage=="kiss" else "fine"]["elapsed_ms"]:.2f} ms'
  fig.text(.055+col*.315,.175,elapsed,fontsize=12,weight='bold')
  fig.text(.055+col*.315,.137,f'独立 RMSE {v["rmse_m"]*100:.2f} cm  ·  重叠率 {v["source_overlap"]*100:.2f}%',fontsize=12)
 fig.text(.045,.947,'第 5 轮实录查询：检索 → 粗配准 → 精配准',fontsize=23,weight='bold')
 fig.text(.045,.9,'使用 11 月 10 日变化场景预建图（133 KF） · C2 确认后 C1 独立验证通过 · 离线重放成功',fontsize=13,color='#547087')
 fig.text(.045,.86,'灰色：同一候选子图     洋红：同一当前查询云     上排同一投影与缩放，下排同一 XY 局部区域',fontsize=12)
 fig.text(.045,.08,f'自有 v2（195 维）仅检索地点；KISS 求完整 6DoF 初值；GICP 细化。候选 KF {selected["keyframe"]}，查询 scan_id={s["scan_id"]}。',fontsize=11)
 fig.text(.045,.043,'耗时来自本次离线实测，不是原实车耗时；RMSE/重叠率采用相同独立门控定义，不是绝对位姿误差或完整性能排名。',fontsize=11,color='#64748b')
 export(fig,out,'02_trial05_registration_triptych')
 result={'stages_source':str(folder/'stages.json'),'map_uuid':s['map_uuid'],'same_point_counts':[len(p) for p in sources],
  'common_projection':{'yaw_rad':yaw,'elevation_rad':elev,'limits':[lo.tolist(),hi.tolist()]},'common_xy_detail_bounds':roi.tolist(),
  'elapsed_ms':{'kiss':selected['coarse']['elapsed_ms'],'gicp':selected['fine']['elapsed_ms']},'independent_validation':vals,
  'c2_confirmed':True,'c1_valid':True}
 save_json(out/'02_registration_statistics.json',result);return result

def main():
 p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--registration',type=Path)
 args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
 result={'factor_graph':factors(args.trial,args.output),'closure':closure(args.trial,args.output)}
 if args.registration:result['registration']=registration(args.registration,args.output)
 save_json(args.output/'figure_checks.json',result)
 print('PASS exported figures as PNG / SVG / PDF')
if __name__=='__main__':main()
