#!/usr/bin/env python3
"""Summarize completed offline checks; no source bags or maps are modified."""
import hashlib
import json
from pathlib import Path
import sys
from collections import Counter
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'

def read(path):return json.loads(path.read_text())
def rows(path):return [json.loads(s) for s in path.read_text().splitlines()]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def trial(folder):
    result=read(folder/'result.json');samples=rows(folder/'samples.jsonl')
    valid=[s for s in samples if s['pose_valid']]
    first=valid[0] if valid else None
    return {'output':str(folder.relative_to(ROOT)),'passed':result['passed'],'logged_inputs':len(samples),
            'processed_pairs':result.get('processed_pairs'),'cancelled_pairs':result.get('cancelled_pairs',0),
            'valid_samples':len(valid),'states':dict(Counter(s['state'] for s in samples)),
            'first_valid_query_relative_seconds':(first['stamp_ns']-samples[0]['stamp_ns'])*1e-9 if first else None,
            'first_valid_index':first['index'] if first else None,
            'reasons':dict(Counter(s['reason'] for s in samples)),
            'localizer_exit':result['localizer_exit'],'error':result.get('error'),
            'node_peak_rss_kib':result.get('node_peak_rss_kib'),'wall_seconds':result.get('wall_seconds'),
            'solve_ms_p50_p95_max':result.get('solve_ms_p50_p95_max'),
            'public_publishers':result.get('public_publishers'),
            'source_database_sha256':result['source_database_sha256'],
            'source_manifest_sha256':result['source_manifest_sha256'],
            'result_sha256':sha(folder/'result.json'),'samples_sha256':sha(folder/'samples.jsonl')}

def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: summarize completed offline results');return 0
    cases={'old_turn_C2':OUT/'turn/c2','old_straight_C2':OUT/'straight/c2',
           'old_straight_C2_candidates10':OUT/'straight/c2_candidates10',
           'new_straight_original_node':OUT/'straight/c2_new_map',
           'new_straight_optimized_first_attempt':OUT/'straight/c2_new_map_optimized',
           'new_straight_C2_recovery':OUT/'straight/c2_new_map_recovery',
           'new_turn_given_initial_C1':OUT/'turn/c1_new_map_optimized'}
    report={'trials':{k:trial(v) for k,v in cases.items()},'same_cohort':read(OUT/'same_cohort_map_comparison/summary.json'),
            'backfill':read(OUT/'backfill_new_map_recovery/summary.json'),
            'new_map':read(OUT/'new_local_map/result.json'),
            'time_selection':read(HERE/'frozen_selection.json'),'regressions':read(HERE/'optimized_ros_regressions.json'),
            'secondary_straight_diagnostic':read(HERE/'secondary_straight_self_fit.json'),
            'ground_truth_accuracy_validated':False,'physical_time_verified':False,'real_VALID_landmark_created':False,
            'production_configuration_changed':False,'hardware_started':False}
    with (HERE/'final_summary.json').open('x') as stream:stream.write(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(2,2,figsize=(16,10),layout='constrained')
    for y,(key,label) in enumerate([('old_straight_C2','旧图 / 原 C2'),('new_straight_C2_recovery','新局部图 / C2 → C1')]):
        samples=rows(ROOT/report['trials'][key]['output']/'samples.jsonl');origin=samples[0]['stamp_ns']
        t=np.array([(s['stamp_ns']-origin)*1e-9 for s in samples]);v=np.array([s['pose_valid'] for s in samples])
        axes[0,0].scatter(t,np.full(len(t),y),c=np.where(v,'#2c8388','#abb3bd'),marker='|',s=250)
        cancelled=[(s['stamp_ns']-origin)*1e-9 for s in samples if s.get('input_outcome','acknowledged')!='acknowledged']
        if cancelled:axes[0,0].scatter(cancelled,[y]*len(cancelled),marker='x',color='#b44949',s=90,zorder=5)
        first=report['trials'][key]['first_valid_query_relative_seconds']
        axes[0,0].text(2,y-.23,'未完成初始化' if first is None else f'首次有效 {first:.2f} s；{v.sum()} 个有效采样',fontsize=11)
    axes[0,0].set_yticks([0,1],['旧图','新局部图']);axes[0,0].set_ylim(1.5,-.7)
    axes[0,0].set(title='独立直行包：无人工初值，Tag 不参与定位',xlabel='原查询流传感器时间 / s',xlim=(0,90))
    axes[0,0].text(.02,.01,'灰：无有效定位  青：C1确认有效  红×：初始化超时取消',transform=axes[0,0].transAxes,fontsize=9)
    cohort=report['same_cohort'];keys=['local_livo','old_map_c1','new_map_c1'];values=[cohort['cases'][k]['holdout_rmse_px']['p95'] for k in keys]
    bars=axes[0,1].bar(range(3),values,color=['#7f9d8b','#9da5af','#2c8388'],width=.6);axes[0,1].bar_label(bars,fmt='%.2f')
    axes[0,1].set_xticks(range(3),['局部三融合','旧图 C1','新局部图 C1'])
    axes[0,1].set(title=f'相同 {len(cohort["heldout_image_indices"])} 张转向留出图像',ylabel='固定 Tag 重投影 RMS P95 / px')
    back=report['backfill'];path=OUT/'backfill_new_map_recovery/straight_frozen_turn_landmark_reprojection.json'
    if path.exists():
        frames=read(path)['frame_errors'];origin=rows(OUT/'straight/c2_new_map_recovery/samples.jsonl')[0]['stamp_ns']
        axes[1,0].plot([(r['stamp_ns']-origin)*1e-9 for r in frames],[r['rmse_px'] for r in frames],color='#8174a8',lw=1)
        axes[1,0].set(title='只在转向包拟合 Tag，冻结后检查直行包',xlabel='直行包传感器时间 / s',ylabel='跨访问回投 RMS / px')
    else:
        axes[1,0].text(.1,.5,'无足够独立直行 C1 支持，不能验证地标复用',transform=axes[1,0].transAxes)
    content=(OUT/'new_local_map/snapshot/map.pcd').read_bytes();head,payload=content.split(b'DATA binary\n',1)
    assert b'FIELDS x y z intensity' in head
    points=np.frombuffer(payload,dtype='<f4').reshape(-1,4)
    axes[1,1].scatter(points[::8,0],points[::8,1],s=.3,c='#b6bdc5',alpha=.35,rasterized=True)
    for name,color,folder in [('转向：建图访问','#2c8388',OUT/'turn/c1_new_map_optimized'),('直行：独立访问','#c88b37',OUT/'straight/c2_new_map_recovery')]:
        poses=np.array([s['pose_map_imu']['xyz'] for s in rows(folder/'samples.jsonl') if s['pose_valid']])
        if len(poses):axes[1,1].plot(poses[:,0],poses[:,1],c=color,label=name,lw=1.5)
    axes[1,1].axis('equal');axes[1,1].legend(fontsize=9)
    axes[1,1].set(title='新局部点云图及已确认轨迹（俯视）',xlabel='地图 X / m',ylabel='地图 Y / m')
    fig.suptitle('地图对照与独立访问验证：原图保留，所有几何与安全门槛保持',fontsize=16)
    fig.supxlabel('条件性 +110 ms 假设；非整层地图；无独立位置真值；时偏未部署、无真实 VALID Tag 地标',fontsize=10)
    fig.savefig(HERE/'map_and_independent_visit.png',dpi=120);fig.savefig(HERE/'map_and_independent_visit.svg')
    print(json.dumps({k:{f:v[f] for f in ['passed','logged_inputs','processed_pairs','cancelled_pairs','valid_samples','first_valid_query_relative_seconds']} for k,v in report['trials'].items()},indent=2))
    return 0
if __name__=='__main__':raise SystemExit(main())
