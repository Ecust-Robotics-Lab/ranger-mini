"""Reuse wheel feedback or prepare a receive-only driver for manual capture."""
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2]
NODE=ROOT/'ws_base/install_recording_rx/ranger_base/lib/ranger_base/ranger_base_node'


def driver_command():
    return [str(NODE),'--ros-args','--params-file',str(ROOT/'config/base_mini_v2.yaml'),
            '-p','receive_only:=true','-p','publish_odom_tf:=false']


def wheel_source(publishers, sample, now_ns):
    if not publishers:return 'owned_receive_only'
    if len(publishers)!=1 or publishers[0]['type']!='nav_msgs/msg/Odometry' or publishers[0]['reliability']!='RELIABLE':
        raise RuntimeError('wheel需要唯一RELIABLE的nav_msgs/Odometry发布者；保留现有节点，未启动第二个驱动')
    if (sample is None or sample['frame']!='odom' or sample['child']!='base_link'
            or not 0 <= (now_ns-sample['stamp_ns'])*1e-9 <= .5):
        raise RuntimeError('已有wheel发布者但未收到新鲜odom/base_link数据；未替换现有驱动')
    return 'external'


def discover():
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init();node=rclpy.create_node('fast_livo_capture_wheel_probe')
    sample=None
    def receive(m):
        nonlocal sample
        sample={'frame':m.header.frame_id,'child':m.child_frame_id,
                'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec}
    node.create_subscription(Odometry,'/wheel/odom',receive,10)
    try:
        end=time.monotonic()+2.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        publishers=[{'node':p.node_name,'namespace':p.node_namespace,'type':p.topic_type,
                     'reliability':p.qos_profile.reliability.name,'gid':list(p.endpoint_gid)}
                    for p in node.get_publishers_info_by_topic('/wheel/odom')]
        return {'source':wheel_source(publishers,sample,node.get_clock().now().nanoseconds),
                'publishers':publishers,'sample':sample}
    finally:node.destroy_node();rclpy.shutdown()


def check_other_drivers():
    # CAN device ownership crosses DDS domains. Never start a duplicate driver.
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():continue
        try:
            argv=(item/'cmdline').read_bytes().split(b'\0')
            if argv and Path(argv[0].decode()).name=='ranger_base_node':
                raise RuntimeError('已有底盘进程PID '+item.name+'，但本录包域没有wheel；请统一ROS域后再录制，未接管或停止它')
        except (OSError,UnicodeError):pass


def can_state():
    try:return json.loads(subprocess.check_output(['ip','-json','-details','link','show','can0'],text=True))[0]
    except subprocess.CalledProcessError as exc:raise RuntimeError('未发现can0，请检查USB-CAN连接') from exc


def prepare_can(before):
    data=before.get('linkinfo',{}).get('info_data',{})
    if 'UP' in before.get('flags',[]):
        if data.get('state')!='ERROR-ACTIVE' or data.get('bittiming',{}).get('bitrate')!=500000:
            raise RuntimeError('can0已启用但不是500k ERROR-ACTIVE；保留当前配置，不自动重置')
        return {'before':before,'enabled_by_capture':False,'left_up_after_capture':True}
    command=['sudo','-n','/usr/sbin/ip','link','set','can0','up','type','can','bitrate','500000']
    result=subprocess.run(command,capture_output=True,text=True)
    if result.returncode:
        raise RuntimeError('无法免密启用can0；请在本机完成固定CAN命令的sudo配置：'+result.stderr.strip())
    after=can_state();data=after.get('linkinfo',{}).get('info_data',{})
    if 'UP' not in after.get('flags',[]) or data.get('state')!='ERROR-ACTIVE' or data.get('bittiming',{}).get('bitrate')!=500000:
        raise RuntimeError('can0启用后状态异常，未启动底盘驱动')
    return {'before':before,'after':after,'enable_command':command,'enabled_by_capture':True,
            'left_up_after_capture':True,'rollback_command':['sudo','-n','/usr/sbin/ip','link','set','can0','down']}


def prepare():
    result=discover()
    if result['source']=='owned_receive_only':
        check_other_drivers()
        if not os.access(NODE,os.X_OK):raise RuntimeError('缺少录包只接收驱动；先执行 bash scripts/build_base_recording.sh --execute')
        result['can']=prepare_can(can_state())
        result['driver_command']=driver_command()
    return result
