#!/usr/bin/env python3
"""Summarize actual trajectory ablations, without changing measurements or thresholds."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
EV=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-motion-geometry-20261104'
spec=importlib.util.spec_from_file_location('motion_diagnostic',ROOT/'scripts/fast_livo2/tags/diagnose_motion_geometry.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)

def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: actual-data static diagnostics and plots');return 0
    config=json.loads(Path((EV/'config_path.txt').read_text().strip()).read_text())
    observations=d.analysis.rows(ROOT/'outputs/fast-livo-tag140-sync-diag02-screen-20261029/observations.jsonl')
    images=json.loads((EV/'cohort.json').read_text());by_index={r['index']:r for r in observations}
    cohort=[by_index[i] for i in images];origin=observations[0]['stamp_ns']
    datasets=[]
    for folder in ('comparison_final','comparison_bias'):
        for file in (OUT/folder).glob('*.json'):
            if file.name=='summary.json':continue
            result=json.loads(file.read_text());case=result['case']
            records=d.load_native(case['path']) if case['format']=='native_db' else d.analysis.rows(case['path'])
            cams,support=d.analysis.associate(d.common.PoseHistory(records),cohort,d.common.matrix(config['T_imu_camera']),case['offset_ns'])
            assert all(s['available'] for s in support)
            errors,positive=d.analysis.projected_errors(d.common.matrix(result['global']['pose']),np.linalg.inv(cams),cohort,config['camera'])
            corners=np.asarray([r['observation']['corners'] for r in cohort]);predicted=errors+corners
            windows=[]
            for start in range(0,116,5):
                selected=[i for i,r in enumerate(cohort) if start*10**9<=r['stamp_ns']-origin<(start+5)*10**9]
                if len(selected)<45:continue
                observed=corners[selected];delta=observed-np.median(observed,axis=0)
                spread=np.sqrt(np.mean(np.sum(delta**2,axis=2),axis=1))
                if np.percentile(spread,95)>.2 or spread.max()>.5:continue
                estimated=predicted[selected];delta=estimated-np.median(estimated,axis=0)
                jitter=np.sqrt(np.mean(np.sum(delta**2,axis=2),axis=1))
                windows.append({'start_s':start,'frames':len(selected),'observed_corner_spread_px':d.common.quantiles(spread),
                                'predicted_corner_spread_px':d.common.quantiles(jitter)})
            datasets.append({'case':case,'global_holdout_px':result['global']['holdout_rmse_px'],
                             'initial_threshold_pass':result['global']['frames_passing_initial_3px_rms_6px_max'],
                             'static_windows':windows})
    (EV/'static_windows.json').write_text(json.dumps(datasets,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(2,2,figsize=(16,10),layout='constrained')
    selected=[('livo_0','三融合：原配置', '#dc5b52'),('livo_120000000','三融合：+120 ms','#cf9831'),
              ('lio_110000000','LiDAR–IMU 诊断：+110 ms','#31878d'),('bias_livo_110000000','三融合：初始零偏对照 / +110 ms','#6171b9')]
    for name,label,color in selected:
        folder='comparison_bias' if name.startswith('bias') else 'comparison_final'
        r=json.loads((OUT/folder/(name+'.json')).read_text());frames=r['global']['frame_errors']
        times=np.asarray([(v['stamp_ns']-origin)*1e-9 for v in frames])
        values=np.asarray([v['rmse_px'] for v in frames])
        # Do not draw invented residuals across intervals where the Tag is absent.
        cuts=np.flatnonzero(np.diff(times)>.25)+1
        axes[0,0].plot(np.insert(times,cuts,np.nan),np.insert(values,cuts,np.nan),label=label,color=color,lw=.8,alpha=.85)
    axes[0,0].set(xlabel='采样时间 / s',ylabel='角点重投影 RMS / px',ylim=(0,40),title='整段固定 Tag：逐帧残差（显示范围 0–40 px）')
    axes[0,0].legend(fontsize=8)
    lookup={v['case']['name']:v for v in datasets}
    values=[lookup[n]['global_holdout_px']['p95'] for n,_,_ in selected]
    bars=axes[0,1].bar(range(4),values,color=[c for _,_,c in selected]);axes[0,1].set_xticks(range(4),['原三融合','三融合\n+120 ms','LiDAR–IMU\n+110 ms','初始零偏对照\n三融合 +110 ms'])
    axes[0,1].bar_label(bars,fmt='%.2f');axes[0,1].set(ylabel='留出帧 RMS 的 P95 / px',title='同一 274 留出帧；后两项为诊断实验')
    sweep=json.loads((OUT/'time_sweep/summary.json').read_text());scores=sweep['scores']
    axes[1,0].plot([r['offset_ns']/1e6 for r in scores],[r['training_cost'] for r in scores],color='#31878d')
    axes[1,0].axvline(110,color='#777',ls='--');axes[1,0].set(xlabel='假设有效时间 = header + 偏移 / ms',ylabel='训练 Huber 坐标代价',title='固定 LiDAR–IMU 轨迹：训练选择约 +110 ms')
    for name,label,color in selected:
        windows=lookup[name]['static_windows']
        axes[1,1].plot([w['start_s'] for w in windows],[w['predicted_corner_spread_px']['p95'] for w in windows],'o-',label=label,color=color)
    windows=lookup['livo_0']['static_windows']
    axes[1,1].plot([w['start_s'] for w in windows],[w['observed_corner_spread_px']['p95'] for w in windows],'k.--',label='实际图像角点离散度')
    axes[1,1].set(xlabel='图像稳定窗口起点 / s',ylabel='窗口内角点离散度 P95 / px',title='实测图像稳定时：预测投影仍有额外波动')
    axes[1,1].legend(fontsize=8)
    fig.suptitle('Tag 动态几何诊断：时间关联有影响，单独补偿仍未解决三融合残差',fontsize=17)
    fig.supxlabel('现有实采包的离线实验；同次采集时间留出；未证明物理同步或绝对精度，未部署时偏/零偏改动',fontsize=10)
    fig.savefig(EV/'motion_geometry_diagnostic.png',dpi=120)
    fig.savefig(EV/'motion_geometry_diagnostic.svg')
    (EV/'summary.json').write_text(json.dumps({'common_cohort':len(cohort),'cases':datasets,'physical_time_verified':False,'runtime_changed':False,'real_VALID_landmark_created':False},indent=2)+'\n')
    return 0

if __name__=='__main__':raise SystemExit(main())
