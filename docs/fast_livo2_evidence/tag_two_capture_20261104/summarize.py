#!/usr/bin/env python3
"""Plot the independent checks, retaining failed C2 results and scope limitations."""
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'

def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: summarize completed independent-capture checks');return 0
    evaluations={}
    for name in ['turn','straight']:
        for mode in ['lio','livo']:
            evaluations[name+'_'+mode]=json.loads((OUT/name/(mode+'_evaluation/summary.json')).read_text())
    c2={name:json.loads((OUT/name/'c2/result.json').read_text()) for name in ['turn','straight']}
    c2['straight_candidates10']=json.loads((OUT/'straight/c2_candidates10/result.json').read_text())
    for name,result in c2.items():
        folder=OUT/('straight/c2_candidates10' if name.endswith('candidates10') else name+'/c2')
        samples=[json.loads(s) for s in (folder/'samples.jsonl').read_text().splitlines()]
        valid=[s for s in samples if s['pose_valid']]
        result['first_valid_query_relative_seconds']=(valid[0]['stamp_ns']-samples[0]['stamp_ns'])*1e-9 if valid else None
    backfill=json.loads((OUT/'backfill_control/summary.json').read_text())
    report={'tag_detection':{},'frozen_time_selection':json.loads((HERE/'frozen_selection.json').read_text()),
            'evaluations':evaluations,'C2':c2,'backfill':backfill,
            'physical_time_verified':False,'real_VALID_landmark_created':False,'runtime_changed':False,
            'ground_truth_accuracy_validated':False}
    for name,screen in [('turn','fast-livo-tag140-turn03-screen-20261104'),('straight','fast-livo-tag140-straight04-screen-20261104')]:
        s=json.loads((ROOT/'outputs'/screen/'summary.json').read_text());report['tag_detection'][name]={'source':s['source'],'counts':s['counts'],'identities':s['identities']}
    (HERE/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(2,2,figsize=(16,10),layout='constrained')
    grid=json.loads((OUT/'turn/time_sweep/summary.json').read_text())['scores']
    axes[0,0].plot([r['offset_ns']/1e6 for r in grid],[r['fit']['training_huber_coordinate_cost_mean'] for r in grid],color='#2c8388')
    axes[0,0].axvline(110,ls='--',color='#b77b23',label='冻结候选 +110 ms')
    axes[0,0].set(title='新转向包：仅训练帧选择时偏',xlabel='图像有效时间 = header + 偏移 / ms',ylabel='训练 Huber 坐标代价')
    axes[0,0].legend()
    labels=['转向\nLiDAR–IMU','前后\nLiDAR–IMU','转向\n三融合','前后\n三融合']
    pairs=[evaluations[k] for k in ['turn_lio','straight_lio','turn_livo','straight_livo']]
    x=np.arange(4)
    for offset,shift,color,label in [(0,-.18,'#a2a9b4','原时间设置'),(110000000,.18,'#2c8388','+110 ms 假设')]:
        values=[next(r for r in p['scores'] if r['offset_ns']==offset)['fit']['holdout_rmse_px']['p95'] for p in pairs]
        bars=axes[0,1].bar(x+shift,values,.36,color=color,label=label);axes[0,1].bar_label(bars,fmt='%.2f',fontsize=9)
    axes[0,1].set_xticks(x,labels);axes[0,1].set(ylabel='留出帧重投影 RMS P95 / px',title='每组同一 240 留出帧；前后包未参与时偏选择')
    axes[0,1].legend()
    for y,(key,label) in enumerate([('turn','转向 / 原5候选'),('straight','前后 / 原5候选'),('straight_candidates10','前后 / 10候选实验')]):
        r=c2[key];first=r['first_valid_query_relative_seconds'];axes[1,0].barh(y,89.4,color='#b9bec8',height=.55)
        if first is not None:axes[1,0].barh(y,89.4-first,left=first,color='#2c8388',height=.55)
        axes[1,0].text(2,y,('约 %.2f s 首次有效；%d有效采样'%(first,r['valid_samples'])) if first is not None else '895对处理；未完成初始化',va='center',fontsize=10)
    axes[1,0].set_yticks(range(3),['转向 / 原5候选','前后 / 原5候选','前后 / 10候选实验']);axes[1,0].invert_yaxis()
    axes[1,0].set(xlabel='查询流传感器时间 / s',title='旧图纯 LiDAR 候选 C2：Tag 未参与定位',xlim=(0,90))
    values=[backfill['turn_local_same_cohort_heldout_rmse_px']['p95'],backfill['turn_heldout_rmse_px']['p95']]
    bars=axes[1,1].bar([0,1],values,color=['#2c8388','#8174a8'],width=.55);axes[1,1].bar_label(bars,fmt='%.2f')
    axes[1,1].set_xticks([0,1],['局部三融合轨迹','匹配旧地图后的 C1 位姿'])
    axes[1,1].set(ylabel='Tag 重投影 RMS P95 / px',title='转向包：完全相同的160留出帧对照')
    fig.suptitle('两份新包验证：+110 ms 得到支持；完整地图定位与 Tag 补录仍未通过全部检查',fontsize=16)
    fig.supxlabel('原包只读；同包时间留出与跨包条件分开记录；Tag不是独立位置真值；未部署时偏或生成VALID地标',fontsize=10)
    fig.savefig(HERE/'independent_capture_results.png',dpi=120);fig.savefig(HERE/'independent_capture_results.svg')
    print(json.dumps({'selected_offset_ns':report['frozen_time_selection']['turn_selected_offset_ns'],
        'C2':{k:{f:v[f] for f in ['passed','processed_pairs','valid_samples','first_valid_query_relative_seconds']} for k,v in c2.items()},
        'physical_time_verified':False,'real_VALID_landmark_created':False},indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
