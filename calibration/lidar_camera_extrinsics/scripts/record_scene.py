#!/usr/bin/python3
"""Run only after the user coordinates exclusive camera time; never launches drivers."""
import argparse, datetime, os, signal, subprocess, sys, time
from pathlib import Path
from common import ROOT,LIDAR,IMAGE,output,save,read

def main():
    p=argparse.ArgumentParser();p.add_argument('scene',choices=['scene_01','scene_02','scene_03','validation_01']);p.add_argument('--seconds',type=float,default=8);a=p.parse_args()
    if not 5<=a.seconds<=10:raise ValueError('Use 5..10 seconds')
    # This interactive gate is deliberately present because capture needs user confirmation.
    expected='RECORD '+a.scene
    if input('确认已协调内参 Agent、硬件由用户启动、相机雷达与板完全静止。输入 '+expected+' 开始录制：\n').strip()!=expected:return 2
    os.environ['ROS_LOG_DIR']=str(ROOT/'logs/ros')
    import rclpy
    from rclpy.qos import ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('c2_record_preflight');deadline=time.monotonic()+8
    infos={}
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.2);infos={t:node.get_publishers_info_by_topic(t) for t in (LIDAR,IMAGE)}
        if all(infos.values()):break
    snapshot={t:[{'node':i.node_name,'type':i.topic_type,'reliability':str(i.qos_profile.reliability),'durability':str(i.qos_profile.durability)} for i in v] for t,v in infos.items()}
    ok=all(len(infos[t])==1 and infos[t][0].qos_profile.reliability==ReliabilityPolicy.RELIABLE and infos[t][0].topic_type==typ for t,typ in [(LIDAR,'sensor_msgs/msg/PointCloud2'),(IMAGE,'sensor_msgs/msg/Image')])
    node.destroy_node();rclpy.shutdown()
    if not ok:raise ValueError('Require one reliable publisher per topic: '+str(snapshot))
    run=ROOT/'datasets'/a.scene/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');run.mkdir(parents=True,exist_ok=False)
    save(run/'publisher_snapshot.json',snapshot)
    bag=run/'bag';log=(run/'record.log').open('w')
    cmd=['ros2','bag','record','--storage','sqlite3','--max-cache-size','536870912','--qos-profile-overrides-path',str(ROOT/'config/record_qos.yaml'),'-o',str(bag),LIDAR,IMAGE]
    save(run/'capture_manifest.json',{'scene':a.scene,'duration_requested_s':a.seconds,'command':cmd,'user_confirmed':True,'solve_requested':False})
    proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        time.sleep(a.seconds)
    finally:
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=40)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
        log.close()
    if not (bag/'metadata.yaml').exists():raise ValueError('Recorder did not finalize: '+str(run/'record.log'))
    result=subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/calib.py'),'audit',str(bag),'--out',str(run/'bag_audit.json')])
    if result.returncode==0:
        report=read(run/'bag_audit.json')
        for topic in (LIDAR,IMAGE):
            td=report['topics'][topic]
            if not 5<=td['header_duration_s']<=11 or not 45<=td['count']<=115:
                print('采集长度/帧数异常，请检查录制启动或中途退出：',run,file=sys.stderr);return 3
    print('采集与审计结束，未执行求解。请同时保存驱动 FrameID CSV：',run)
    return result.returncode
if __name__=='__main__':
    try:sys.exit(main() or 0)
    except (ValueError,RuntimeError) as e:print('BLOCKED:',e,file=sys.stderr);sys.exit(2)
