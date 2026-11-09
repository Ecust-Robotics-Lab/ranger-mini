#!/usr/bin/env python3
"""Guardian heartbeat for the offline C1 service; no devices or public TF."""
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from ranger_fast_livo_localizer.msg import LocalizerEstimate


class Monitor(Node):
    def __init__(self):
        super().__init__('ranger_localizer_session_monitor')
        self.last = self.received = None
        self.output = Path(os.environ['RANGER_SESSION_HEALTH_FILE'])
        self.create_subscription(LocalizerEstimate, '/fast_livo/localizer/estimate', self.receive, 10)
        self.create_timer(.2, self.tick)

    def receive(self, message):
        self.last, self.received = message, time.monotonic()

    def tick(self):
        alive = self.received is not None and time.monotonic()-self.received < 1.
        healthy = alive and not self.last.reason.startswith('terminal_')
        data = {'monotonic': time.monotonic(), 'checks': {'probe_live': True, 'localizer_alive': healthy},
                'localizer_state': self.last.state if self.last else None,
                'reason': self.last.reason if self.last else 'waiting for C1 heartbeat'}
        tmp = self.output.with_suffix('.tmp')
        tmp.write_text(json.dumps(data)+'\n')
        tmp.replace(self.output)


if __name__ == '__main__':
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
