#!/usr/bin/env python3
"""Steady-time session health; navigation readiness remains the existing bridge's job."""
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool


class Monitor(Node):
    def __init__(self):
        super().__init__('fast_livo_navigation_prep_monitor')
        self.output=Path(os.environ['RANGER_SESSION_HEALTH_FILE'])
        self.map_ready=False;self.bridge_received=None;self.ready=False
        self.create_subscription(OccupancyGrid,'/map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray,'/localization_health',self.on_bridge,10)
        self.create_subscription(Bool,'/navigation_ready',lambda m:setattr(self,'ready',m.data),10)
        self.create_timer(.2,self.tick)

    def on_map(self,m):
        self.map_ready=m.header.frame_id=='map' and m.info.width*m.info.height==len(m.data) and len(m.data)>0

    def on_bridge(self,m):
        if m.status:self.bridge_received=time.monotonic()

    def tick(self):
        data={'monotonic':time.monotonic(),'checks':{'probe_live':True,'map_ready':self.map_ready,
              'bridge_alive':self.bridge_received is not None and time.monotonic()-self.bridge_received<1.},
              'navigation_ready':self.ready,'physical_sync_verified':False}
        tmp=self.output.with_suffix('.tmp');tmp.write_text(json.dumps(data)+'\n');tmp.replace(self.output)


if __name__=='__main__':
    rclpy.init();node=Monitor()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
