#!/usr/bin/env python3
"""Isolated offline map + existing C1 adapter/TF bridge. No devices or control."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'ws_nav/src/ranger_localization_bridge'),str(Path(__file__).parent)]
from asset import load_map
from session_manager import Component,read_session,session_live,stop_session,run_guarded
from ranger_localization_bridge.runtime_config import resolve

SESSION=ROOT/'.runtime/sessions/fast_livo_navigation_prep'


def components(folder,geometry):
    manifest=load_map(folder);cfg=resolve(ROOT)
    install=yaml.safe_load(Path(geometry).read_text())['installation']
    extrinsics=['-p','t_base_imu_xyz:='+json.dumps(install['t_base_imu_xyz']),
                '-p','t_base_imu_rpy:='+json.dumps(install['t_base_imu_rpy'])]
    common=['--ros-args','-p','use_sim_time:=true']
    bounds=[]
    for key,param in [('TF_MAX_AGE_S','tf_max_age_s'),('ODOM_MAX_AGE_S','odom_max_age_s'),
                      ('ODOM_HISTORY_S','odom_history_s'),('ODOM_MAX_GAP_S','odom_max_gap_s'),('HEARTBEAT_MAX_AGE_S','heartbeat_max_age_s')]:
        bounds+=['-p',param+':='+cfg[key]]
    specs=[Component('monitor',[sys.executable,str(Path(__file__).with_name('monitor.py'))],ready_key='probe_live'),
        Component('map_server',['/opt/ros/humble/lib/nav2_map_server/map_server',*common,
            '-r','__node:=fast_livo_navigation_map','-p','yaml_filename:='+str(Path(folder).resolve()/'map.yaml'),
            '-p','topic_name:=map','-p','frame_id:=map'],depends=('monitor',),resource='navigation_map'),
        Component('map_lifecycle',['/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager',*common,
            '-r','__node:=fast_livo_navigation_map_lifecycle','-p','autostart:=true',
            '-p','node_names:=[fast_livo_navigation_map]'],depends=('map_server',),ready_key='map_ready')]
    for key,frame in [('imu','fast_livo_imu'),('lidar','livox_frame')]:
        if 't_base_'+key+'_xyz' not in install:continue
        xyz=install['t_base_'+key+'_xyz'];rpy=install['t_base_'+key+'_rpy']
        values=[*xyz,*rpy]
        flags=[v for name,value in zip(('x','y','z','roll','pitch','yaw'),values) for v in ('--'+name,str(value))]
        specs.append(Component('static_'+key,['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',*flags,
            '--frame-id','base_link','--child-frame-id',frame,*common,'-r','__node:=fast_livo_static_'+key],
            depends=('map_lifecycle',),resource='fast_livo_static_'+key))
    specs += [Component('adapter',[sys.executable,'-m','ranger_localization_bridge.fast_livo_localizer_adapter',*common,*extrinsics,
            '-p','expected_map_uuid:='+manifest['map_uuid'],'-p','expected_manifest_sha256:='+manifest['source_manifest_sha256']],
            depends=('map_lifecycle',),resource='localization_backend'),
        Component('bridge',[sys.executable,'-m','ranger_localization_bridge.bridge_node',*common,
            '-p','adapter_backend:=external','-p','latch_on_innovation_failure:=true',*bounds],
            depends=('adapter',),ready_key='bridge_alive',resource='localization_bridge')]
    return specs


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group();mode.add_argument('--execute',action='store_true');mode.add_argument('--status',action='store_true');mode.add_argument('--stop',action='store_true')
    p.add_argument('--navigation-map',type=Path);p.add_argument('--geometry-config',type=Path,default=ROOT/'config/fast_livo2/navigation_prep.yaml')
    p.add_argument('--domain',type=int,default=219);args=p.parse_args()
    if args.status:
        _,state=read_session(SESSION);print(json.dumps({'live':bool(session_live(state)),'details':state},indent=2));return 0
    if args.stop:return 0 if stop_session(SESSION) else 2
    if not args.navigation_map:p.error('--navigation-map is required')
    if not 1<=args.domain<=232:p.error('isolated domain must be 1..232')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',PYTHONDONTWRITEBYTECODE='1')
    os.environ['PYTHONPATH']=str(ROOT/'ws_nav/src/ranger_localization_bridge')+':'+os.environ.get('PYTHONPATH','')
    specs=components(args.navigation_map,args.geometry_config)
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','domain':args.domain,'localhost_only':True,'offline_only':True,
            'requires':'same-sample-time C1 estimate, wheel odometry and /clock from offline inputs',
            'hardware_started':False,'physical_sync_verified':False,'components':[asdict(c) for c in specs]},indent=2));return 0
    if session_live(read_session(SESSION)[1]):raise RuntimeError('offline bridge session already active; inspect --status')
    import rclpy
    rclpy.init();probe=rclpy.create_node('fast_livo_navigation_prep_occupancy_probe')
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(probe,timeout_sec=.05)
        topics=['/map','/tf','/tf_static','/localization/estimate','/navigation_ready','/cmd_vel','/base/safe_cmd_vel']
        occupied=[t for t in topics if probe.get_publishers_info_by_topic(t)]
        if occupied:raise RuntimeError('isolated offline outputs already occupied: '+', '.join(occupied))
    finally:probe.destroy_node();rclpy.shutdown()
    return run_guarded(SESSION,specs,startup_timeout=25,fail_on_health_loss=True,health_loss_grace=1.)


if __name__=='__main__':raise SystemExit(main())
