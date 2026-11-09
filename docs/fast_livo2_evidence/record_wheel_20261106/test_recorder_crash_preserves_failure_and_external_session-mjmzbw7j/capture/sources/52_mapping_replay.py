#!/usr/bin/env python3
"""Play only original sensor topics from an audited header-clock view."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from session_manager import atomic_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--view', required=True, type=Path)
    parser.add_argument('--folder', required=True, type=Path)
    args = parser.parse_args()
    stopping = [False]
    def stop(*_): stopping[0] = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(sig, stop)
    command = ['ros2', 'bag', 'play', str(args.view), '--clock', '100', '--rate', '1', '--delay', '3',
        '--qos-profile-overrides-path', str(Path(__file__).resolve().parents[2]/'config/mid360/bag_qos.yaml'),
        '--topics', '/livox/lidar', '/livox/imu', '/front/image_sync', '/front/image', '/front/camera_info']
    atomic_json(args.folder/'replay_command.json', command)
    child = subprocess.Popen(command, start_new_session=True)
    try:
        while not stopping[0] and child.poll() is None: time.sleep(.05)
        if not stopping[0]:
            atomic_json(args.folder/'replay_result.json', {'exit': child.returncode, 'rate': 1., 'hardware_started': False})
            if child.returncode: return 2
            # Let the frontend finish callbacks already delivered before EOF.
            end = time.monotonic()+.3
            while time.monotonic() < end and not stopping[0]: time.sleep(.01)
            os.kill(os.getppid(), signal.SIGTERM)
            while not stopping[0]: time.sleep(.05)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try: child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                child.terminate()
                try: child.wait(timeout=2)
                except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=2)
    return 0


if __name__ == '__main__': raise SystemExit(main())
