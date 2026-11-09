#!/usr/bin/env python3
"""Managed sensor-only FAST-LIVO2 + RViz session. Dry-run unless --execute."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from session_manager import Component,run_guarded,read_session,session_live,stop_session
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import resolve,validate_fast_livo_config


def specs(cfg, visualization='rviz', foxglove_port=8765):
    share=Path(os.environ['RANGER_LIVO_TIMESHARE'])
    cal=ROOT/'ws_camera'
    lidar=ROOT/'ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'
    writer=cal/'install/timeshare_writer/lib/timeshare_writer/timeshare_writer_node'
    camera=cal/'install/hik_camera_trigger_driver/lib/hik_camera_trigger_driver/hik_camera_trigger_driver_node'
    cparams=cfg['FAST_LIVO_CAMERA_CONFIG']
    components = [
      Component('probe',['python3',str(ROOT/'scripts/fast_livo2/live_monitor.py')],ready_key='probe_live'),
      Component('lidar',[str(lidar),'--ros-args','-p','xfer_format:=0','-p','multi_topic:=0','-p','data_src:=0',
        '-p','publish_freq:=10.0','-p','output_data_type:=0','-p','frame_id:=livox_frame',
        '-p','user_config_path:='+cfg['FAST_LIVO_LIDAR_CONFIG'],
        '-p','cmdline_input_bd_code:=livox0000000001','-r','__node:=livox_lidar_publisher'],
        depends=('probe',),ready_key='lidar',start_requires=('lidar_unclaimed',),resource='livox_lidar'),
      Component('writer',[str(writer),'--ros-args','-p','timeshare_path:='+str(share),
        '-p','lidar_reliability:=reliable','-p','lidar_queue_depth:=5',
        '-p','csv_path:='+str(share.parent/'timeshare_writer.csv')],depends=('lidar',),requires=('lidar',),ready_key='writer',resource='camera_timeshare'),
      Component('camera',['bash','-c','cd -- "$RANGER_LIVO_DEVICE_CWD" && exec "$@"','camera-device',str(camera),'--ros-args','--params-file',str(cparams),
        '-p','timeshare_path:='+str(share),'-p','sync_csv_path:='+str(share.parent/'camera_sync.csv'),
        '-r','__node:=hik_camera_trigger'],depends=('writer',),requires=('writer',),ready_key='camera',
        start_requires=('camera_unclaimed',),resource='hik_camera'),
      Component('frontend',[str(ROOT/'ws_fast_livo/install/fast_livo/lib/fast_livo/fastlivo_mapping'),
        '--ros-args','--params-file',cfg['FAST_LIVO_CONFIG'],
        '-p','use_sim_time:=false','--log-level','warn'],depends=('camera',),requires=('lidar','camera'),
        ready_key='frontend',start_requires=('frontend_unclaimed',),resource='fast_livo_frontend')]
    if visualization in ('rviz', 'both'):
        components.append(Component('rviz',['/opt/ros/humble/lib/rviz2/rviz2','-d',cfg['FAST_LIVO_RVIZ_CONFIG'],
            '--ros-args','-p','use_sim_time:=false'],depends=('frontend',),critical=False,resource='fast_livo_rviz'))
    if visualization in ('foxglove', 'both'):
        components.append(Component('foxglove', ['/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge',
            '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/foxglove.yaml'),
            '-p', 'port:='+str(foxglove_port)], depends=('frontend',), critical=False,
            resource='fast_livo_foxglove'))
    return components


def sensor_environment(cfg, share_dir):
    """Shared environment for the preview and recording entries; no device is opened."""
    os.environ.update(ROS_DOMAIN_ID=cfg['FAST_LIVO_ROS_DOMAIN_ID'],
                      ROS_LOCALHOST_ONLY=cfg['FAST_LIVO_LOCALHOST_ONLY'], OMP_NUM_THREADS='4',
                      QT_QPA_PLATFORM='xcb', RANGER_LIVO_TIMESHARE=str(share_dir/'timeshare'),
                      RANGER_LIVO_DEVICE_CWD=str(share_dir))
    sdk=ROOT/'ws_camera/install/hik_camera_trigger_driver/lib'
    os.environ['LD_LIBRARY_PATH']=':'.join([str(ROOT/'ws_fast_livo/deps_install/lib'),str(sdk),
        str(sdk/'ThirdParty'),os.environ.get('LD_LIBRARY_PATH','')])
    os.environ['GENICAM_GENTL64_PATH']=str(sdk)


def check_sensor_processes():
    """Refuse device owners in other DDS domains too; never signal their processes."""
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():continue
        try:
            names=[Path(x.decode()).name for x in (item/'cmdline').read_bytes().split(b'\0')[:2] if x]
            if any(n in ('livox_ros_driver2_node','timeshare_writer_node','hik_camera_trigger_driver_node',
                         'hik_camera_ros2_driver_node','fastlivo_mapping') for n in names):
                raise RuntimeError('Existing device/frontend process; coordinate its owner first: '+item.name)
        except (OSError,UnicodeDecodeError):pass


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    a=p.add_mutually_exclusive_group();a.add_argument('--execute',action='store_true');a.add_argument('--status',action='store_true');a.add_argument('--stop',action='store_true')
    p.add_argument('--runtime-config')
    p.add_argument('--visualization', choices=('rviz', 'foxglove', 'both', 'none'), default='rviz')
    p.add_argument('--foxglove-port', type=int, default=8765)
    args=p.parse_args();directory=ROOT/'.runtime/sessions/fast_livo_live'
    if not 1 <= args.foxglove_port <= 65535:p.error('foxglove-port must be 1..65535')
    if args.status:
        _,s=read_session(directory);print(json.dumps({'live':bool(session_live(s)),'details':s},indent=2));raise SystemExit(0)
    if args.stop:raise SystemExit(0 if stop_session(directory) else 2)
    if args.runtime_config:os.environ['RANGER_RUNTIME_CONFIG']=args.runtime_config
    cfg=resolve(ROOT)
    validate_fast_livo_config(cfg['FAST_LIVO_CONFIG'])
    # Each start uses a private shared file; never truncates the historical ~/timeshare.
    import tempfile
    if args.execute:
        directory.mkdir(parents=True,exist_ok=True)
        if session_live(read_session(directory)[1]):raise SystemExit('Existing live session; use --status/--stop')
        check_sensor_processes()
        share_dir=Path(tempfile.mkdtemp(prefix='sensors-',dir=directory))
    else:share_dir=directory/'sensors-NEW'
    sensor_environment(cfg, share_dir)
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','domain':int(cfg['FAST_LIVO_ROS_DOMAIN_ID']),'runtime_config':cfg['RANGER_RUNTIME_CONFIG'],'components':[asdict(c) for c in specs(cfg,args.visualization,args.foxglove_port)]},indent=2));raise SystemExit(0)
    (share_dir/'manifest.json').write_text(json.dumps({'sync_protocol':'v2 even committed','domain':int(cfg['FAST_LIVO_ROS_DOMAIN_ID']),'runtime_config':cfg['RANGER_RUNTIME_CONFIG'],'frontend_config':cfg['FAST_LIVO_CONFIG'],'hardware_authorization':'Requires user authorization for this sensor session; no base/navigation',
        'calibration':'PARTIAL trial; zero time offset','initial_network':subprocess.check_output(['ip','-br','address'],text=True)},indent=2)+'\n')
    raise SystemExit(run_guarded(directory,specs(cfg,args.visualization,args.foxglove_port),startup_timeout=70,fail_on_health_loss=True,health_loss_grace=2))
