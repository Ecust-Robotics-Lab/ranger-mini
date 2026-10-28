#!/usr/bin/env python3
"""Managed-session heartbeat monitor; writes only the guardian's private health file."""
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from ranger_fast_livo_backend.msg import BackendEstimate


class Monitor(Node):
    def __init__(self):
        super().__init__('ranger_backend_session_monitor')
        self.last = None; self.received = None
        self.output = Path(os.environ['RANGER_SESSION_HEALTH_FILE'])
        self.create_subscription(BackendEstimate, '/fast_livo/backend/estimate', self.estimate, 10)
        self.create_timer(.2, self.tick)

    def estimate(self, message):
        self.last, self.received = message, time.monotonic()

    def tick(self):
        alive = self.received is not None and time.monotonic()-self.received < 1.
        healthy = alive and self.last.state != self.last.ERROR
        data = {'monotonic': time.monotonic(), 'checks': {'probe_live': True, 'backend_healthy': healthy},
                'backend_state': self.last.state if self.last else None,
                'reason': self.last.reason if self.last else 'waiting for backend heartbeat'}
        tmp = self.output.with_suffix('.tmp'); tmp.write_text(json.dumps(data)+'\n'); tmp.replace(self.output)


if __name__ == '__main__':
    rclpy.init(); node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
