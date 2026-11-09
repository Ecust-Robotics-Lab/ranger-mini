#!/usr/bin/env python3
"""Read-only FAST/wheel diagnostics. Candidate clock/extrinsics never attest field sync."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation
import yaml


def load_streams(bag):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    arrays={}
    for name,kind,tag in [('/wheel/odom',Odometry,'wheel'),('/fast_livo/odometry',Odometry,'fast'),('/livox/imu',Imu,'imu')]:
        rows=[]
        for db in meta['relative_file_paths']:
            with sqlite3.connect('file:'+str((bag/db).resolve())+'?mode=ro',uri=True) as con:
                tid=con.execute('select id from topics where name=?',(name,)).fetchone()[0]
                for record,data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp,id',(tid,)):
                    m=deserialize_message(data,kind);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                    if tag=='imu':
                        value=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,
                               m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]
                    else:
                        p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular
                        value=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z]
                    rows.append((stamp,record,value))
        rows.sort(key=lambda r:r[1])
        arrays[tag+'_stamp']=np.array([r[0] for r in rows],dtype=np.int64)
        arrays[tag+'_record']=np.array([r[1] for r in rows],dtype=np.int64)
        arrays[tag+'_values']=np.array([r[2] for r in rows])
    return arrays


def median_ns(values):
    values=np.sort(values)
    return int(values[len(values)//2])


def fit_lag(t, signal, wheel_t, wheel_rate, low, high):
    keep=(t>low)&(t<high)&(t>wheel_t[0]+.16)&(t<wheel_t[-1]-.16)
    t=t[keep];g=signal[keep];gc=g-g.mean();results=[]
    for lag in np.arange(-.15,.151,.002):
        w=np.interp(t+lag,wheel_t,wheel_rate);wc=w-w.mean()
        if np.std(w)<.01:continue
        gain=float(wc@gc/(wc@wc));bias=float(g.mean()-gain*w.mean())
        residual=g-(gain*w+bias)
        results.append({'lag_seconds':round(float(lag),6),'gain':gain,'bias_rad_s':bias,
                        'rms_rad_s':float(np.sqrt(np.mean(residual**2))),
                        'correlation':float(np.corrcoef(g,w)[0,1]),'samples':len(t)})
    if not results:raise ValueError('insufficient angular excitation for lag analysis')
    return min(results,key=lambda r:r['rms_rad_s'])


def diagnostic_feedback(values):
    """Infer moving vs spinning from recorded twist; not a replacement for raw CAN modes."""
    moving=np.linalg.norm(values[:,7:10],axis=1)>1e-6
    yaw=np.unwrap(Rotation.from_quat(values[:,3:7]).as_euler('xyz')[:,2])
    turns=np.diff(yaw)*np.where(moving[1:],-1.,1.)
    new_yaw=np.r_[yaw[0],yaw[0]+np.cumsum(turns)]
    delta=np.diff(values[:,:2],axis=0);c=np.cos(yaw[:-1]);s=np.sin(yaw[:-1])
    local=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]))
    local[moving[1:],1]*=-1
    c=np.cos(new_yaw[:-1]);s=np.sin(new_yaw[:-1])
    step=np.column_stack((c*local[:,0]-s*local[:,1],s*local[:,0]+c*local[:,1]))
    xy=np.vstack((values[0,:2],values[0,:2]+np.cumsum(step,axis=0)))
    rate=values[:,-1]*np.where(moving,-1.,1.)
    return xy,new_yaw,rate


def spin_axis(imu_t, gyro, wheel_t, wheel, low=-np.inf, high=np.inf):
    rate=np.interp(imu_t,wheel_t,wheel[:,-1])
    speed=np.interp(imu_t,wheel_t,np.linalg.norm(wheel[:,7:10],axis=1))
    keep=(speed<.005)&(abs(rate)>.2)&(imu_t>low)&(imu_t<high)
    if keep.sum()<50:raise ValueError('insufficient stationary spin for axis diagnostic')
    coeff=np.linalg.lstsq(np.column_stack((rate[keep],np.ones(keep.sum()))),gyro[keep],rcond=None)[0]
    scale=float(np.linalg.norm(coeff[0]))
    return coeff[0]/scale,{'samples':int(keep.sum()),'gyro_per_wheel_spin_scale':scale,
                          'bias_xyz_rad_s':coeff[1].tolist()}


def rotation_candidate(fast_t, fast, wheel_t, wheel, axis, low, high):
    velocity=np.gradient(fast[:,:3],fast_t,axis=0)
    velocity_imu=Rotation.from_quat(fast[:,3:7]).inv().apply(velocity)
    vx=np.interp(fast_t,wheel_t,wheel[:,7]);vy=np.interp(fast_t,wheel_t,wheel[:,8])
    wz=np.interp(fast_t,wheel_t,wheel[:,-1])
    keep=(abs(vx)>.25)&(abs(vy)<.01)&(abs(wz)<.025)&(fast_t>low)&(fast_t<high)
    if keep.sum()<20:raise ValueError('insufficient straight motion')
    forward=np.median(velocity_imu[keep]/vx[keep,None],axis=0)
    forward-=axis*(forward@axis);forward/=np.linalg.norm(forward)
    rotation=np.column_stack((forward,np.cross(axis,forward),axis)).T
    return rotation,{'samples':int(keep.sum()),'rpy_rad':Rotation.from_matrix(rotation).as_euler('xyz').tolist(),
                     'rpy_deg':Rotation.from_matrix(rotation).as_euler('xyz',degrees=True).tolist()}


def bracket_count(source_ns, target_ns):
    indices=np.searchsorted(source_ns,target_ns)
    exact=(indices<len(source_ns))&(source_ns[np.minimum(indices,len(source_ns)-1)]==target_ns)
    inside=(indices>0)&(indices<len(source_ns))
    good=exact.copy()
    ids=indices[inside]
    good[inside]|=(source_ns[ids]-source_ns[ids-1])<=100_000_000
    return {'total':len(target_ns),'bracketed_without_extrapolation':int(good.sum()),
            'max_bracket_gap_seconds':.1}


def analyze(x):
    coarse=median_ns(x['imu_record']-x['imu_stamp']);anchor=int(x['imu_stamp'][0])
    it=(x['imu_stamp']-anchor)*1e-9;wt=(x['wheel_stamp']-(anchor+coarse))*1e-9
    ft=(x['fast_stamp']-anchor)*1e-9;wheel=x['wheel_values'];fast=x['fast_values'];gyro=x['imu_values'][:,:3]
    xy,yaw,rate=diagnostic_feedback(wheel)
    axis,axis_quality=spin_axis(it,gyro,wt,wheel)
    projected=gaussian_filter1d(gyro@axis,3);end=min(it[-1],wt[-1],ft[-1]);mid=end/2
    fits={}
    for name,low,high in [('full',5,end-5),('first_half',5,mid),('second_half',mid,end-5)]:
        fits[name]=fit_lag(it,projected,wt,rate,low,high)
    raw=fit_lag(it,projected,wt,wheel[:,-1],5,end-5)
    rotations={}
    for name,low,high in [('full',0,end),('first_half',0,mid),('second_half',mid,end)]:
        local_axis,local_quality=spin_axis(it,gyro,wt,wheel,low,high)
        r,info=rotation_candidate(ft,fast,wt,wheel,local_axis,low,high)
        info.update(spin_axis_in_imu=local_axis.tolist(),spin_samples=local_quality['samples'])
        rotations[name]=info
        if name=='full':rotation=r
    lag=fits['full']['lag_seconds'];candidate_offset=coarse+round(lag*1e9)
    residual=x['imu_record']-x['imu_stamp']-coarse
    fy=np.unwrap(Rotation.from_quat(fast[:,3:7]).as_euler('xyz')[:,2])
    wy=np.unwrap(Rotation.from_quat(wheel[:,3:7]).as_euler('xyz')[:,2])
    summaries={}
    for tag in ['wheel','fast']:
        v=x[tag+'_values'];stamp=x[tag+'_stamp']
        summaries[tag]={'messages':len(v),'span_seconds':float((stamp[-1]-stamp[0])*1e-9),
                        'path_length_m':float(np.linalg.norm(np.diff(v[:,:3],axis=0),axis=1).sum()),
                        'header_strictly_increasing':bool(np.all(np.diff(stamp)>0))}
    # Compare estimator trajectories after one initial rigid alignment, without scale fitting.
    nominal_lidar=np.array([.109,0.,.600]);tbi=nominal_lidar+rotation@np.array([.011,.02329,-.04412])
    rwi=Rotation.from_quat(fast[:,3:7]);rwb=rwi*Rotation.from_matrix(rotation).inv()
    base=fast[:,:3]-rwb.apply(np.tile(tbi,(len(fast),1)))
    base=base[:,:2]-base[0,:2]
    heading=rwb.as_euler('xyz')[0,2]
    align=np.array([[np.cos(heading),-np.sin(heading)],[np.sin(heading),np.cos(heading)]])
    compared={}
    for name,positions in [('raw',wheel[:,:2]),('diagnostic_sign_corrected',xy)]:
        relative=(positions-positions[0])@align.T
        target=(x['fast_stamp']+candidate_offset-(anchor+coarse))*1e-9
        keep=(target>=wt[0])&(target<=wt[-1])
        sampled=np.column_stack([np.interp(target[keep],wt,relative[:,k]) for k in range(2)])
        error=np.linalg.norm(sampled-base[keep],axis=1)
        compared[name]={'median_disagreement_m':float(np.median(error)),'p95_disagreement_m':float(np.percentile(error,95)),
                        'end_disagreement_m':float(error[-1]),'not_ground_truth':True}
    report={'schema_version':1,'kind':'wheel_alignment_diagnostic','streams':summaries,
        'receipt_coarse_offset_ns':coarse,'receipt_residual_ms_p0_p1_p50_p99_p100':np.percentile(residual*1e-6,[0,1,50,99,100]).tolist(),
        'raw_angular_fit':raw,'sign_corrected_angular_fits':fits,
        'candidate_sensor_to_host_offset_ns':candidate_offset,'candidate_offset_valid_for_this_capture_only':True,
        'physical_sync_verified':False,'live_clock_mapping_enabled':False,
        'spin_axis_in_imu':axis.tolist(),'spin_axis_tilt_deg':float(np.rad2deg(np.arccos(axis[2]))),
        'spin_axis_quality':axis_quality,'extrinsic_rotation_candidates':rotations,
        'extrinsic_translation_candidate_m':tbi.tolist(),'extrinsics_field_calibrated':False,
        'wheel_brackets_original_epochs':bracket_count(x['wheel_stamp'],x['fast_stamp']),
        'wheel_brackets_diagnostic_mapping':bracket_count(x['wheel_stamp'],x['fast_stamp']+candidate_offset),
        'yaw_changes_rad':{'fast':float(fy[-1]-fy[0]),'raw_wheel':float(wy[-1]-wy[0]),
                           'diagnostic_wheel':float(yaw[-1]-yaw[0])},
        'trajectory_comparison':compared,
        'limitations':['No raw CAN motion-mode stream in this bag; old-wheel repair is diagnostic and infers moving vs spin from twist.',
                       'Motion time fits include transport/feedback lag; neither receipt time nor a fitted lag certifies physical synchronization.',
                       'Axis assumes planar spin; forward axis assumes signed wheel vx during straight driving. Same-capture split is a consistency check.',
                       'No APE/RPE or absolute navigation accuracy; no global map/C1/TF acceptance is claimed.']}
    arrays={'imu_t':it,'wheel_t':wt,'fast_t':ft,'gyro_projected':gyro@axis,'wheel_rate_corrected':rate,
            'wheel_xy_corrected':xy,'wheel_yaw_corrected':yaw,'fast_base_xy_candidate':base}
    return report,arrays


def plot(x,a,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(3,1,figsize=(13,10),sharex=True)
    ax[0].plot(a['imu_t'],a['gyro_projected'],label='IMU projected yaw + bias',lw=.7)
    ax[0].plot(a['wheel_t'],x['wheel_values'][:,-1],label='raw wheel',lw=.7)
    ax[0].plot(a['wheel_t'],a['wheel_rate_corrected'],label='feedback sign corrected',lw=.7)
    ax[0].set_ylabel('rad/s');ax[0].legend()
    fy=np.unwrap(Rotation.from_quat(x['fast_values'][:,3:7]).as_euler('xyz')[:,2])
    wy=np.unwrap(Rotation.from_quat(x['wheel_values'][:,3:7]).as_euler('xyz')[:,2])
    ax[1].plot(a['fast_t'],fy-fy[0],label='FAST estimated yaw')
    ax[1].plot(a['wheel_t'],wy-wy[0],label='raw wheel yaw')
    ax[1].plot(a['wheel_t'],a['wheel_yaw_corrected']-a['wheel_yaw_corrected'][0],label='diagnostic corrected yaw')
    ax[1].set_ylabel('rad');ax[1].legend()
    speed=np.linalg.norm(np.gradient(x['fast_values'][:,:3],a['fast_t'],axis=0),axis=1)
    ax[2].plot(a['fast_t'],speed,label='FAST position speed',lw=.7)
    ax[2].plot(a['wheel_t'],np.linalg.norm(x['wheel_values'][:,7:10],axis=1),label='wheel speed',lw=.7)
    ax[2].set(xlabel='relative seconds; diagnostic epoch mapping',ylabel='m/s');ax[2].legend()
    fig.suptitle('Same-capture diagnostics: no ground truth / physical sync acceptance')
    fig.tight_layout();fig.savefig(output/'motion_diagnostics.png',dpi=120);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bag',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise ValueError('output must be a new directory')
    x=load_streams(args.bag.resolve());report,arrays=analyze(x)
    report['source_bag']=str(args.bag.resolve())
    report['source_metadata_sha256']=hashlib.sha256((args.bag/'metadata.yaml').read_bytes()).hexdigest()
    report['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output/'streams.npz',**x)
    np.savez_compressed(args.output/'diagnostic_arrays.npz',**arrays)
    (args.output/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    plot(x,arrays,args.output);print(json.dumps(report,indent=2))


if __name__=='__main__':main()
