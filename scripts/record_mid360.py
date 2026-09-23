#!/usr/bin/env python3
"""Finite SQLite3 recording of raw Livox, available base state and TF; no drivers started."""
import argparse,json,pathlib,subprocess,tempfile,time,signal,os
ROOT=pathlib.Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--label', choices=['static','manual-loop'], default='static', help='Recording label only; never commands motion')
    a=p.parse_args()
    if not 1<=a.seconds<=600:p.error('seconds must be in [1,600]')
    topics=['/livox/lidar','/livox/imu','/tf','/tf_static','/system_state','/motion_state','/actuator_state','/wheel/odom','/battery_state','/livox/imu_si']
    if not a.execute:print('DRY-RUN: sqlite3',a.seconds,'seconds',topics);return 0
    import rclpy
    rclpy.init();node=rclpy.create_node('mid360_recorder_preflight')
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    graph={}
    for topic in topics:
        graph[topic]=[dict(node=e.node_namespace.rstrip('/')+'/'+e.node_name,type=e.topic_type,reliability=e.qos_profile.reliability.name,durability=e.qos_profile.durability.name,depth=e.qos_profile.depth) for e in node.get_publishers_info_by_topic(topic)]
    node.destroy_node();rclpy.shutdown()
    for topic,typ in [('/livox/lidar','livox_ros_driver2/msg/CustomMsg'),('/livox/imu','sensor_msgs/msg/Imu')]:
        if len(graph[topic])!=1 or graph[topic][0]['type']!=typ:raise RuntimeError('Expected one raw publisher: '+repr(graph))
    folder=pathlib.Path(tempfile.mkdtemp(prefix='mid360-'+a.label+'-',dir=ROOT/'bags'))
    (folder/'graph_before.json').write_text(json.dumps(graph,indent=2))
    (folder/'driver_config.json').write_bytes((ROOT/'config/mid360/MID360S_config.json').read_bytes())
    cmd=['ros2','bag','record','-s','sqlite3','-o',str(folder/'raw'),'--qos-profile-overrides-path',str(ROOT/'config/mid360/bag_qos.yaml')]+topics
    (folder/'record_command.json').write_text(json.dumps(cmd,indent=2))
    print('BAG_RUN='+str(folder),flush=True)
    env=os.environ.copy();env['ROS_LOG_DIR']=str(folder/'ros_logs')
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    child=None;forced=False
    try:
        with (folder/'record.log').open('x') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            child.wait(timeout=a.seconds)
    except (subprocess.TimeoutExpired,KeyboardInterrupt):pass
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=20)
            except subprocess.TimeoutExpired:forced=True;child.terminate();child.wait(timeout=10)
    (folder/'record_result.json').write_text(json.dumps(dict(exit=child.returncode,forced=forced,requested_seconds=a.seconds),indent=2))
    if forced or child.returncode!=0 or not (folder/'raw/metadata.yaml').exists():return 2
    return subprocess.call(['/usr/bin/python3',str(ROOT/'scripts/check_mid360_bag.py'),str(folder/'raw'),'--output',str(folder/'validation.json')])
if __name__=='__main__':raise SystemExit(main())
