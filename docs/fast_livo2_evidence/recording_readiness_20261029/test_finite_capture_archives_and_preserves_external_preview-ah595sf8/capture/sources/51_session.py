#!/usr/bin/env python3
"""Managed passive offline AprilTag subscriber. Never starts image sources."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from session_manager import Component, run_guarded, read_session, session_live, stop_session


def main():
    p = argparse.ArgumentParser(description=__doc__)
    action = p.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true')
    action.add_argument('--status', action='store_true')
    action.add_argument('--stop', action='store_true')
    p.add_argument('--config', type=Path, default=ROOT/'config/fast_livo2/apriltag_camera.json')
    p.add_argument('--family', choices=['tag36h11', 'tagStandard41h12'], default='tag36h11')
    p.add_argument('--sizes-json', help='Override measured corner-boundary sizes, e.g. {"0":0.16}')
    p.add_argument('--image-topic', default='/front/image_sync')
    p.add_argument('--record', type=Path)
    p.add_argument('--domain', type=int, default=189)
    args = p.parse_args()
    if not 1 <= args.domain <= 232:
        p.error('offline domain must be 1..232')
    directory = ROOT/'.runtime/sessions/fast_livo_tags'
    if args.status:
        state = read_session(directory)[1]
        print(json.dumps({'live': bool(session_live(state)), 'details': state}, indent=2))
        return 0
    if args.stop:
        return 0 if stop_session(directory) else 2
    if args.sizes_json is None:
        measured = json.loads((ROOT/'config/fast_livo2/apriltag_sizes.json').read_text())
        args.sizes_json = json.dumps(measured['sizes_m'] if measured['family'] == args.family else {})
    command = [str(ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/tag_observer_node'),
               '--ros-args', '-p', 'use_sim_time:=true', '-p', 'tag_config_path:='+str(args.config.resolve()),
               '-p', 'family:='+args.family, '-p', "tag_sizes_json:='"+args.sizes_json.replace("'", "''")+"'",
               '-p', 'image_topic:='+args.image_topic]
    if args.record:
        command += ['-p', 'record_path:='+str(args.record.resolve())]
    components = [Component('tag_observer', command, resource='fast_livo_tag_observer')]
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'hardware_started': False, 'localhost_only': True,
                          'domain': args.domain, 'components': [asdict(c) for c in components]}, indent=2))
        return 0
    if session_live(read_session(directory)[1]):
        raise RuntimeError('Tag observer session is already live; inspect --status')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    import rclpy
    rclpy.init()
    probe = rclpy.create_node('ranger_tag_occupancy_probe')
    try:
        import time
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.05)
        if probe.get_publishers_info_by_topic('/fast_livo/tags/observations'):
            raise RuntimeError('Tag observation output already owned')
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    return run_guarded(directory, components, startup_timeout=10, fail_on_health_loss=True)


if __name__ == '__main__':
    raise SystemExit(main())
