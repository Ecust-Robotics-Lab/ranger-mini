"""Independent owner watchdog: only revoke enable when an owned task loses its owner."""
import argparse
import json
from pathlib import Path
import time
from workbench.core import ROOT
import sys
sys.path.insert(0,str(ROOT/'scripts'))
from session_manager import same_process

def expired(record,now):
    return bool(record.get('active') and (not same_process(record.get('identity')) or now-record.get('steady',0)>.5))

def main(path):
    import rclpy
    from std_msgs.msg import Bool
    rclpy.init();node=rclpy.create_node('ranger_workbench_owner_watchdog');pub=node.create_publisher(Bool,'/ranger/motion_enable',10)
    record={}
    try:
        while rclpy.ok():
            try:record=json.loads(path.read_text())
            except (OSError,ValueError):pass
            if expired(record,time.monotonic()):pub.publish(Bool(data=False))
            rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:
        if record.get('active'):
            pub.publish(Bool(data=False))
            rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--heartbeat',type=Path,required=True);main(p.parse_args().heartbeat)
