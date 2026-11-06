#!/usr/bin/env python3
"""Real-data plots and original-image crop, no sensor or localization outputs."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
import cv2
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent;OUT=ROOT/'outputs/fast-livo-c1-correction-20261104'
spec=importlib.util.spec_from_file_location('control_data',HERE.parent/'tag_two_capture_20261104/backfill_assessment.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
if sys.argv[1:]!=['--execute']:
    print('DRY-RUN: draw recorded correction controls');raise SystemExit(0)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
summary=json.loads((OUT/'straight_comparison/summary.json').read_text())
rows=[json.loads(s) for s in (OUT/'straight_diagnostics/corrections.jsonl').read_text().splitlines()]
origin=rows[0]['stamp_ns'];time=np.array([(r['stamp_ns']-origin)*1e-9 for r in rows])
fig,axes=plt.subplots(2,2,figsize=(15,9),layout='constrained')
axes[0,0].plot(time,[r['body_residual_from_first_alignment_m']*100 for r in rows],c='#8174a8',lw=1)
axes[0,0].set(title='相对首次对齐：C1 与连续局部轨迹的差异',xlabel='首次 VALID 后的传感器时间 / s',ylabel='机器人原点位置差 / cm')
changed=np.array([r.get('target_changed',False) for r in rows])
axes[0,0].scatter(time[changed],[rows[i]['body_residual_from_first_alignment_m']*100 for i in np.where(changed)[0]],c='#c88b37',s=4,label='目标关键帧集合变化')
axes[0,0].legend(fontsize=9)
values=[[r['geometric_step_translation_m']*100 for r in rows if r.get('consecutive_native_interval') and r['target_changed']==c] for c in [False,True]]
axes[0,1].boxplot(values,labels=['目标集合不变','目标集合切换'],showfliers=True,widths=.45)
axes[0,1].set(title='逐帧几何修正：切换子图不能解释全部波动',ylabel='额外平移修正 / cm')
keys=['pcl_dynamic','pcl_frozen','pcl_aggregate_voxel','gicp_dynamic','local_native']
labels=['原 PCL','固定目标\nPCL','合并降采样\nPCL','已有 GICP','局部三融合']
values=[summary['cases'][c]['heldout_rmse_px']['p95'] for c in keys]
bars=axes[1,0].bar(range(len(keys)),values,color=['#a5adb8','#9b86ad','#bcaa7b','#2c8388','#7f9d8b']);axes[1,0].bar_label(bars,fmt='%.2f')
axes[1,0].set_xticks(range(len(keys)),labels);axes[1,0].set(title=f'相同 {len(summary["heldout_image_indices"])} 张留出图像，单帧控制实验',ylabel='固定 Tag 回投 RMS P95 / px')
fits={c:json.loads((OUT/'straight_comparison'/(c+'_fit.json')).read_text()) for c in ['pcl_dynamic','gicp_dynamic']}
for c,color,label in [('pcl_dynamic','#a5adb8','原 PCL'),('gicp_dynamic','#2c8388','已有 GICP')]:
    f=fits[c]['frame_errors'];axes[1,1].plot([(r['stamp_ns']-origin)*1e-9 for r in f],[r['rmse_px'] for r in f],c=color,lw=.8,label=label)
axes[1,1].legend();axes[1,1].set(title='同一图像集合的完整误差曲线',xlabel='首次 VALID 后的传感器时间 / s',ylabel='回投 RMS / px')
fig.suptitle('C1 修正归因：同扫描、同初值、原几何门槛；生产配置未变',fontsize=15)
fig.supxlabel('内部一致性对照；Tag不是独立位置真值；GICP 1帧未收敛已拒绝；完整运行链另行核验',fontsize=10)
fig.savefig(HERE/'correction_controls.png',dpi=130);fig.savefig(HERE/'correction_controls.svg')
held=set(summary['heldout_image_indices']);example=max((r for r in fits['pcl_dynamic']['frame_errors'] if r['image_index'] in held),key=lambda r:r['rmse_px'])
index=example['image_index'];data=b.collect('straight','c2_new_map_recovery',ROOT/'outputs/fast-livo-tag140-two-capture-20261104/new_local_map/snapshot')
i=next(i for i,r in enumerate(data['observations']) if r['index']==index);row=data['observations'][i];prov=data['attachments'][i]
cfg,_,history,_=b.evaluation.load('straight','livo',110000000);native={r['stamp_ns']:b.d.common.matrix(r['pose']) for r in history.records};E=b.d.common.matrix(cfg['T_imu_camera'])
regs={(r['case'],r['stamp_ns']):r for r in map(json.loads,(OUT/'straight_probe/registrations.jsonl').read_text().splitlines())}
projections={}
for case in ['pcl_dynamic','gicp_dynamic']:
    X=b.d.common.matrix(regs[case,prov['scan_stamp_ns']]['registration']['T_target_source'])
    camera=X@np.linalg.inv(native[prov['scan_stamp_ns']])@b.d.common.matrix(prov['native_interpolation']['pose'])@E
    residual,_=b.d.analysis.projected_errors(b.d.common.matrix(fits[case]['pose']),np.asarray([np.linalg.inv(camera)]),[row],cfg['camera'])
    projections[case]=np.asarray(row['observation']['corners'])+residual[0]
reference=row['observation']['provenance']['image_reference'];database,message_id=reference.split('#')[0],int(reference.rsplit('/',1)[1])
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
con=b.d.common.open_readonly(database)
payload=con.execute('SELECT data FROM messages WHERE id=?',(message_id,)).fetchone()[0];con.close();msg=deserialize_message(payload,Image)
assert msg.encoding=='rgb8' and b.d.common.stamp(msg)==row['stamp_ns']
photo=np.ndarray((msg.height,msg.width,3),dtype=np.uint8,buffer=bytes(msg.data),strides=(msg.step,3,1)).copy()
observed=np.asarray(row['observation']['corners']);all_points=np.concatenate([observed,*projections.values()]);low=np.floor(all_points.min(axis=0)-35).astype(int);high=np.ceil(all_points.max(axis=0)+35).astype(int)
low=np.maximum(low,0);high=np.minimum(high,[msg.width,msg.height])
fig,ax=plt.subplots(figsize=(7,7),layout='constrained');ax.imshow(photo);ax.set_xlim(low[0],high[0]);ax.set_ylim(high[1],low[1])
for points,color,label in [(observed,'#e8c529','图像检测角点'),(projections['pcl_dynamic'],'#ef6a69','PCL轨迹 + 训练段固定Tag'),(projections['gicp_dynamic'],'#46d2d1','GICP轨迹 + 训练段固定Tag')]:
    closed=np.concatenate([points,points[:1]]);ax.plot(closed[:,0],closed[:,1],c=color,lw=1.7,marker='o',ms=4,label=label)
ax.legend(fontsize=9);ax.set_xlabel('原图像素 X');ax.set_ylabel('原图像素 Y')
ax.set_title(f'实际图像 {index}：原PCL留出误差最大帧示例\n独立训练常量、相同原图与模型；统计见完整控制图',fontsize=11)
fig.savefig(HERE/'tag_reprojection_example.png',dpi=160)
(HERE/'image_example_provenance.json').write_text(json.dumps({'selection':'largest original PCL RMS among prespecified heldout frames; illustration only, not a ranking','image_index':index,'image_reference':reference,'raw_stamp_ns':row['stamp_ns'],'effective_stamp_ns':row['stamp_ns']+110000000,'PCL_RMS_px':example['rmse_px'],'GICP_RMS_px':next(r['rmse_px'] for r in fits['gicp_dynamic']['frame_errors'] if r['image_index']==index),'raw_bag_modified':False},indent=2)+'\n')
print(json.dumps({'control_plot':'correction_controls.png','image_example':index,'image_source':reference},indent=2))
