#!/usr/bin/env python3
"""Plot completed full runtime results and frozen-original-Tag cross-visit check."""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent;OUT=ROOT/'outputs/fast-livo-c1-correction-20261104';OLD=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
if sys.argv[1:]!=['--execute']:
    print('DRY-RUN: plot completed runtime checks');raise SystemExit(0)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
s=json.loads((OUT/'runtime_comparison/summary.json').read_text())
fig,axes=plt.subplots(2,2,figsize=(15,9),layout='constrained')
x=np.arange(2)
for method,shift,color,label in [('pcl',-.18,'#a5adb8','原 PCL'),('gicp',.18,'#2c8388','已有 GICP')]:
    values=[s['cases'][capture+'_'+method]['heldout_rmse_px']['p95'] for capture in ['turn','straight']]
    bars=axes[0,0].bar(x+shift,values,.36,color=color,label=label);axes[0,0].bar_label(bars,fmt='%.2f')
axes[0,0].set_xticks(x,['转向：178留出帧','直行：160留出帧']);axes[0,0].set(title='完整运行的共同图像集合，各自训练固定Tag',ylabel='留出 RMS P95 / px');axes[0,0].legend()
keys=['pcl_turn_landmark_to_pcl_straight','legacy_frozen_tag_to_gicp'];values=[s['cross_visit'][k]['rmse_px']['p95'] for k in keys]
bars=axes[0,1].bar([0,1],values,color=['#a5adb8','#2c8388'],width=.6);axes[0,1].bar_label(bars,fmt='%.2f')
axes[0,1].set_xticks([0,1],['原 PCL 查询轨迹','GICP 查询轨迹']);axes[0,1].set(title='原Tag常量不改：相同516个跨访问观测',ylabel='回投 RMS P95 / px')
for key,color,label in [('pcl_turn_landmark_to_pcl_straight','#a5adb8','原 PCL'),('legacy_frozen_tag_to_gicp','#2c8388','已有 GICP')]:
    r=json.loads((OUT/'runtime_comparison'/(key+'.json')).read_text())['frame_errors']
    first=r[0]['stamp_ns'];axes[1,0].plot([(q['stamp_ns']-first)*1e-9 for q in r],[q['rmse_px'] for q in r],c=color,lw=.8,label=label)
axes[1,0].set(title='冻结同一Tag后的逐帧检查',xlabel='共同观测开始后的传感器时间 / s',ylabel='回投 RMS / px');axes[1,0].legend()
for y,(method,path) in enumerate([('pcl',OLD/'straight/c2_new_map_recovery/samples.jsonl'),('gicp',OUT/'straight_gicp_runtime/samples.jsonl')]):
    rows=[json.loads(r) for r in path.read_text().splitlines()];origin=rows[0]['stamp_ns'];time=[(r['stamp_ns']-origin)*1e-9 for r in rows]
    colors=['#2c8388' if r['pose_valid'] else ('#b77b23' if r['state']==2 else '#b9bec7') for r in rows]
    axes[1,1].scatter(time,[y]*len(time),c=colors,marker='|',s=180)
    cancelled=[t for t,r in zip(time,rows) if r.get('input_outcome')=='cancelled_initial_verification_timeout']
    if cancelled:axes[1,1].scatter(cancelled,[y]*len(cancelled),marker='x',c='#b44949',s=70)
    trial=s['trials']['straight_'+method]
    axes[1,1].text(1,y-.2,f"首次有效 {trial['first_valid_query_relative_seconds']:.2f} s；{trial['valid_samples']} 个有效采样",fontsize=10)
axes[1,1].set_yticks([0,1],['原PCL','已有GICP']);axes[1,1].set_ylim(1.5,-.6);axes[1,1].set(xlim=(0,90),xlabel='查询流传感器时间 / s',title='启动时间没有明显改善；短暂无效按原规则拒绝')
fig.suptitle('完整流程验证：几何一致性改善，物理时间与真实Tag启用条件仍待验',fontsize=15)
fig.supxlabel('Tag未用于定位；无人工初值直行查询；原地标常量不重拟合；无独立位置真值；正式默认未改',fontsize=10)
fig.savefig(HERE/'runtime_results.png',dpi=130);fig.savefig(HERE/'runtime_results.svg')
print(json.dumps({'runtime_plot':'runtime_results.png','cross_visit_P95_px':values},indent=2))
