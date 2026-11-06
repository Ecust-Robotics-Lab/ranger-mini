#!/usr/bin/env python3
"""Read-only Tag/raw-gyro relative-rotation timing diagnosis; never calibrates runtime."""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

spec = importlib.util.spec_from_file_location('capture_consistency', Path(__file__).with_name('analyze_capture_consistency.py'))
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)
common = analysis.common


class GyroHistory:
    """Short relative rotations only: no accelerometer/position/SLAM estimation."""
    def __init__(self, stamps, rates, max_gap_ns=20_000_000):
        self.stamps = np.asarray(stamps, dtype=np.int64)
        self.rates = np.asarray(rates, dtype=float)
        if len(self.stamps)<2 or self.rates.shape!=(len(self.stamps),3) or not np.isfinite(self.rates).all():
            raise ValueError('invalid gyro samples')
        dt_ns = np.diff(self.stamps)
        if np.any(dt_ns<=0):
            raise ValueError('gyro epoch is not strictly increasing')
        self.origin = int(self.stamps[0])
        self.seconds = (self.stamps-self.origin)*1e-9
        self.gaps = np.flatnonzero(dt_ns>max_gap_ns)
        increments = Rotation.from_rotvec((self.rates[:-1]+self.rates[1:])*.5*(dt_ns*1e-9)[:,None])
        orientations = [Rotation.identity()]
        for step in increments:
            orientations.append(orientations[-1]*step)
        self.slerp = Slerp(self.seconds, Rotation.concatenate(orientations))

    def supported(self, starts, ends):
        a, b = np.asarray(starts,dtype=np.int64),np.asarray(ends,dtype=np.int64)
        valid = (a>=self.stamps[0]) & (b<=self.stamps[-1]) & (b>a)
        for gap in self.gaps:
            valid &= ~((a<self.stamps[gap+1]) & (b>self.stamps[gap]))
        return valid

    def relative(self, starts, ends, R_imu_camera):
        if not np.all(self.supported(starts,ends)):
            raise ValueError('gyro no bracket, gap, or backwards query; no extrapolation')
        a = self.slerp((np.asarray(starts,dtype=np.int64)-self.origin)*1e-9)
        b = self.slerp((np.asarray(ends,dtype=np.int64)-self.origin)*1e-9)
        # R_C0_C1 = R_I_C^T R_I0_I1 R_I_C, maps later camera into earlier camera.
        return R_imu_camera.T@(a.inv()*b).as_matrix()@R_imu_camera


def relative_candidates(before, after):
    """Both branches retained; no matching on rank or global trajectory."""
    rotations, identities = [], []
    for a in before:
        for b in after:
            rotations.append(common.matrix(a['T_camera_tag'])[:3,:3]@
                             common.matrix(b['T_camera_tag'])[:3,:3].T)
            identities.append([a['hypothesis_id'],b['hypothesis_id']])
    return np.asarray(rotations),identities


def profile(history, pairs, E, offsets):
    starts = np.array([p['start_ns'] for p in pairs],dtype=np.int64)
    ends = np.array([p['end_ns'] for p in pairs],dtype=np.int64)
    common_mask = np.ones(len(pairs),dtype=bool)
    for offset in offsets:
        common_mask &= history.supported(starts+offset,ends+offset)
    used = [p for p,ok in zip(pairs,common_mask) if ok]
    if not used:
        raise ValueError('no common gyro support')
    starts,ends = starts[common_mask],ends[common_mask]
    candidates = np.stack([p['candidates_padded'] for p in used])
    valid = np.array([p['candidate_valid'] for p in used])
    primary = np.array([p['primary_combination'] for p in used])
    scores=[]
    for offset in offsets:
        predicted = history.relative(starts+offset,ends+offset,E)
        errors = predicted[:,None,:,:].transpose(0,1,3,2)@candidates
        angle = np.rad2deg(Rotation.from_matrix(errors.reshape(-1,3,3)).magnitude()).reshape(-1,4)
        angle[~valid]=np.inf
        primary_errors=angle[np.arange(len(used)),primary]
        lower=angle.min(axis=1)
        row={'offset_ns':int(offset),'subsets':{},'offset_is_calibrated':False}
        for subset in ('train','holdout','all'):
            selected=np.array([p['rotation_rich'] and (subset=='all' or p['split']==subset) for p in used])
            row['subsets'][subset]={
                'primary_error_deg':common.quantiles(primary_errors[selected]),
                'branch_lower_bound_deg':common.quantiles(lower[selected]),
                'primary_clipped_mean_deg':float(np.minimum(primary_errors[selected],10).mean()) if selected.any() else None,
                'lower_bound_clipped_mean_deg':float(np.minimum(lower[selected],10).mean()) if selected.any() else None}
        row['pair_errors']=[{'pair_id':p['pair_id'],'primary_error_deg':float(x),'branch_lower_bound_deg':float(y),
                             'best_combination_for_this_offset':int(best)}
                            for p,x,y,best in zip(used,primary_errors,lower,angle.argmin(axis=1))]
        scores.append(row)
    return used,scores


def make_pairs(observations, history, origin, offsets):
    eligible={r['index']:r for r in observations if not r['rejection_reasons']}
    pairs=[]
    reasons=Counter()
    for index,a in sorted(eligible.items()):
        if index%3:
            continue
        b=eligible.get(index+3)
        if b is None:
            reasons['endpoint_not_detected_or_eligible']+=1
            continue
        t0,t1=a['stamp_ns'],b['stamp_ns']
        if not 250_000_000<=t1-t0<=350_000_000:
            reasons['interval_not_about_300ms']+=1
            continue
        block,within=divmod(t0-origin,20_000_000_000)
        if within<500_000_000 or t1-origin>=((block+1)*20_000_000_000-500_000_000):
            reasons['split_boundary_purge']+=1
            continue
        branches=[]
        for row in (a,b):
            branches.append([h for h in row['observation']['hypotheses'] if h['hypothesis_id'] in row['eligible_hypothesis_ids']])
        candidates,ids=relative_candidates(*branches)
        first=[min(h,key=lambda v:v['pixel_rmse'])['hypothesis_id'] for h in branches]
        primary=ids.index(first)
        padded=np.tile(np.eye(3),(4,1,1));padded[:len(candidates)]=candidates
        left=np.searchsorted(history.stamps,t0+min(offsets))
        right=np.searchsorted(history.stamps,t1+max(offsets),side='right')
        rate=np.linalg.norm(history.rates[left:right],axis=1)
        # Fixed diagnostic excitation criterion; independent of the chosen offset/branch.
        rate_p95=float(np.percentile(rate,95)) if len(rate) else 0.
        pairs.append({'pair_id':f'{index}:{index+3}','start_ns':t0,'end_ns':t1,
            'indices':[index,index+3],'split':'holdout' if block%3==2 else 'train',
            'gyro_window_p95_radps':rate_p95,'rotation_rich':rate_p95>.1,
            'branch_combinations':ids,'primary_combination':primary,
            'candidates_padded':padded.tolist(),'candidate_valid':[i<len(candidates) for i in range(4)]})
    return pairs,dict(reasons)


def image_static_bias(observations, history, origin, duration_s):
    """Use measured corner stability, never a planned motion schedule or SLAM pose."""
    blocks=[];training_samples=[]
    for start in range(0,int(duration_s)-4,5):
        lo,hi=origin+start*10**9,origin+(start+5)*10**9
        rows=[r for r in observations if lo<=r['stamp_ns']<hi and not r['rejection_reasons']]
        if len(rows)<45:
            continue
        stamps=[r['stamp_ns'] for r in rows]
        if max(np.diff(stamps))>150000000:
            continue
        corners=np.asarray([r['observation']['corners'] for r in rows])
        delta=corners-np.median(corners,axis=0)
        rms=np.sqrt(np.mean(np.sum(delta**2,axis=2),axis=1))
        if np.percentile(rms,95)>.2 or rms.max()>.5:
            continue
        # The entire +/-250 ms sweep stays inside each static image window.
        inside=(history.stamps>=lo+500000000)&(history.stamps<hi-500000000)
        rates=history.rates[inside]
        if len(rates)<100 or np.max(rates.std(axis=0))>.02:
            continue
        split='holdout' if (start//20)%3==2 else 'train'
        blocks.append({'start_s':start,'end_s':start+5,'split':split,'frames':len(rows),
            'corner_spread_px':common.quantiles(rms),'gyro_samples':len(rates),
            'median_gyro_radps':np.median(rates,axis=0).tolist(),'std_gyro_radps':rates.std(axis=0).tolist()})
        if split=='train':training_samples.extend(rates)
    bias=np.median(training_samples,axis=0).tolist() if training_samples else None
    return {'bias_radps':bias,'training_samples':len(training_samples),'blocks':blocks,
            'selection':'5 s windows; >=45 frames; corner RMS spread P95<=0.2/max<=0.5 px; gyro std<=0.02 rad/s; 0.5 s edges purged',
            'method':'componentwise median of training-only image-static gyro samples',
            'thresholds_field_calibrated':False,'runtime_bias_changed':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--screen',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','output':str(args.output),'scope':'diagnostic only, no clock/map update'}))
        return 0
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu,PointCloud2
    from nav_msgs.msg import Odometry
    args.output.mkdir(parents=True,exist_ok=False)
    screen=json.loads((args.screen/'summary.json').read_text())
    capture=args.capture.resolve(); database=capture/'raw/raw_0.db3'
    if str(capture)!=screen['capture'] or common.sha_file(database)!=screen['source']['sha256']:
        raise ValueError('capture/screen identity mismatch')
    before=database.stat()
    manifest=json.loads((capture/'manifest.json').read_text())
    archived=next(r for r in manifest['source_files'] if r['source'].endswith('/apriltag_camera.json'))
    config_path=capture/archived['file']
    if common.sha_file(config_path)!=archived['sha256']:
        raise ValueError('archived calibration modified')
    config=json.loads(config_path.read_text());E=common.matrix(config['T_imu_camera'])[:3,:3]
    conn=common.open_readonly(database)
    topics={name:key for key,name in conn.execute('SELECT id,name FROM topics')}
    imu=[]
    for mid,data in conn.execute('SELECT id,data FROM messages WHERE topic_id=? ORDER BY id',(topics['/livox/imu'],)):
        m=deserialize_message(data,Imu);w=m.angular_velocity
        imu.append({'message_id':mid,'stamp_ns':common.stamp(m),'gyro':[w.x,w.y,w.z]})
    native=common.pose_records(conn,topics['/fast_livo/odometry'],deserialize_message,Odometry)
    lidar=[common.stamp(deserialize_message(data,PointCloud2)) for data, in conn.execute(
        'SELECT data FROM messages WHERE topic_id=? ORDER BY id',(topics['/livox/lidar'],))]
    conn.close()
    import csv
    with (capture/'sensor_assets/camera_sync.csv').open() as f:camera=list(csv.DictReader(f))
    with (capture/'sensor_assets/timeshare_writer.csv').open() as f:writer=list(csv.DictReader(f))
    timing,links=analysis.analyze_sync(camera,writer,lidar)
    (args.output/'software_timing.json').write_text(json.dumps(timing,indent=2)+'\n')
    (args.output/'camera_writer_links.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in links))
    observations=analysis.rows(args.screen/'observations.jsonl')
    frames=analysis.rows(args.screen/'frames.jsonl');origin=frames[0]['stamp_ns']
    history=GyroHistory([r['stamp_ns'] for r in imu],[r['gyro'] for r in imu])
    offsets=list(range(-250_000_000,250_000_001,10_000_000))
    pairs,reasons=make_pairs(observations,history,origin,offsets)
    used,scores=profile(history,pairs,E,offsets)
    if not all(row['subsets']['train']['primary_clipped_mean_deg'] is not None for row in scores):
        raise ValueError('no rotation-rich training intervals; time offset unobservable')
    best_primary=min(scores,key=lambda row:row['subsets']['train']['primary_clipped_mean_deg'])
    best_lower=min(scores,key=lambda row:row['subsets']['train']['lower_bound_clipped_mean_deg'])
    static=image_static_bias(observations,history,origin,(frames[-1]['stamp_ns']-origin)*1e-9+0.1)
    corrected=None;corrected_scores=None
    if static['bias_radps'] is not None:
        corrected_history=GyroHistory(history.stamps,history.rates-np.asarray(static['bias_radps']))
        corrected_pairs,corrected_scores=profile(corrected_history,pairs,E,offsets)
        if [p['pair_id'] for p in corrected_pairs]!=[p['pair_id'] for p in used]:
            raise ValueError('bias sensitivity changed cohort')
        chosen=min(corrected_scores,key=lambda row:row['subsets']['train']['primary_clipped_mean_deg'])
        lower_chosen=min(corrected_scores,key=lambda row:row['subsets']['train']['lower_bound_clipped_mean_deg'])
        corrected={'static_bias_diagnostic':static,'training_selected_primary_offset_ns':chosen['offset_ns'],
            'training_selected_lower_bound_offset_ns':lower_chosen['offset_ns'],
            'zero_offset':{k:v for k,v in next(r for r in corrected_scores if r['offset_ns']==0).items() if k!='pair_errors'},
            'primary_selected':{k:v for k,v in chosen.items() if k!='pair_errors'},
            'lower_bound_selected':{k:v for k,v in lower_chosen.items() if k!='pair_errors'}}
    # Motion summaries are data-derived descriptions, never the planned driving schedule.
    blocks=[]
    native_history=common.PoseHistory(native)
    for start in range(0,120,5):
        lo,hi=origin+start*10**9,origin+(start+5)*10**9
        samples=[r for r in imu if lo<=r['stamp_ns']<hi]
        images=[r for r in observations if lo<=r['stamp_ns']<hi]
        speeds=[native_history.lookup(r['stamp_ns']).get('interval_speed_mps') for r in images]
        gyro=common.quantiles([np.linalg.norm(r['gyro']) for r in samples])
        speed=common.quantiles(speeds)
        label='insufficient_pose_support'
        if gyro.get('p95',0)>.1:label='rotation_present'
        elif speed.get('p95',0)>.03:label='low_rotation_translation_by_frontend'
        elif speed.get('count',0):label='low_motion_by_gyro_and_frontend'
        blocks.append({'start_s':start,'end_s':start+5,'label':label,'gyro_norm_radps':gyro,
                       'conditional_frontend_speed_mps':speed,'tag_frames':len(images)})
    result={'kind':'raw_gyro_and_tag_relative_rotation_diagnostic','source_sha256':screen['source']['sha256'],
        'source_files_sha256':{str(p):common.sha_file(p) for p in
                              (args.screen/'summary.json',args.screen/'observations.jsonl',config_path)},
        'imu_samples':len(imu),'gyro_max_gap_ns':20_000_000,'gyro_bias_subtracted':[0.,0.,0.],
        'source_frame_note':'native Livox gyro axes per driver; configured IMU-to-camera rotation, not inferred from frame_id',
        'image_origin_ns':origin,'offset_definition':'t_effective=t_image_header+offset',
        'common_pairs':len(used),'pair_rejections':reasons,'no_common_gyro_support':len(pairs)-len(used),
        'rotation_rich_pairs':dict(Counter(p['split'] for p in used if p['rotation_rich'])),
        'diagnostic_thresholds':{'gyro_window_p95_radps':.1,'clipped_score_deg':10.,'thresholds_field_calibrated':False},
        'training_selected_primary_offset_ns':best_primary['offset_ns'],
        'training_selected_branch_lower_bound_offset_ns':best_lower['offset_ns'],
        'training_profile_range_deg':{
            name:float(np.ptp([r['subsets']['train'][name] for r in scores]))
            for name in ('primary_clipped_mean_deg','lower_bound_clipped_mean_deg')},
        'zero_offset':next({k:v for k,v in r.items() if k!='pair_errors'} for r in scores if r['offset_ns']==0),
        'primary_selected':{k:v for k,v in best_primary.items() if k!='pair_errors'},
        'lower_bound_selected':{k:v for k,v in best_lower.items() if k!='pair_errors'},
        'image_static_bias_sensitivity':corrected,
        'software_missing_recorded_scans':timing['recorded_lidar_scans_missing_from_writer'],
        'motion_blocks':blocks,'physical_time_verified':False,'clock_mapping_changed':False,
        'frontend_trajectory_used_for_timing_score':False,'frontend_replayed':False,
        'limitations':['Single capture, time-block holdout is not an independent visit',
            'Primary branches ranked by pixel error; may switch planar modes',
            'Lower bound selects branch pair separately against gyro at each offset; optimistic, not a consistent pose track',
            'No gyro bias or extrinsic recalibration; exposure phase not externally measured',
            'Near-constant angular rate does not identify time offset; inspect profile and multiple motion blocks',
            'Rotation-only timing evidence cannot validate translation, global position, or VALID landmarks']}
    for name,data in [('summary',result),('pairs',used),('offset_profile',scores)]:
        (args.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    if corrected_scores is not None:
        (args.output/'bias_corrected_offset_profile.json').write_text(json.dumps(corrected_scores,indent=2)+'\n')
    (args.output/'raw_gyro.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in imu))
    after=database.stat()
    if (after.st_size,after.st_mtime_ns)!=(before.st_size,before.st_mtime_ns):
        raise ValueError('source changed during analysis')
    print(json.dumps(result,indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
