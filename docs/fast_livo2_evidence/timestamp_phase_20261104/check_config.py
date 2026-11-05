#!/usr/bin/env python3
"""Isolated parameter-file smoke check; no devices, playback or motion."""
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess

os.environ['ROS_DOMAIN_ID'] = '211'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

import rclpy
from rclpy.context import Context
from rclpy.node import Node
import yaml

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def main():
    meta = json.loads((HERE/'before.json').read_text())
    backup = Path(meta['backup'])
    config = ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml'
    old = yaml.safe_load((backup/config.relative_to(ROOT)).read_text())
    new = yaml.safe_load(config.read_text())
    assert old['/**']['ros__parameters']['time_offset']['img_time_offset'] == 0.0
    assert new['/**']['ros__parameters']['time_offset']['img_time_offset'] == 0.1
    comparison = copy.deepcopy(new)
    comparison['/**']['ros__parameters']['time_offset']['img_time_offset'] = 0.0
    assert comparison == old, 'unexpected parameter change'
    context = Context()
    node = None
    try:
        rclpy.init(args=['--ros-args', '--params-file', str(config)], context=context)
        node = Node('ranger_timing_config_check', context=context,
                    automatically_declare_parameters_from_overrides=True,
                    start_parameter_services=False, enable_rosout=False)
        assert node.get_parameter('time_offset.img_time_offset').value == 0.1
        for key in ['imu_time_offset', 'lidar_time_offset', 'exposure_time_init']:
            assert node.get_parameter('time_offset.'+key).value == 0.0
        assert node.get_parameter('common.require_livo').value
        assert node.get_parameter('publish.native_tf').value is False
    finally:
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
    dry = json.loads((HERE/'record_dry_run.log').read_text())
    assert dry['action'] == 'DRY-RUN' and dry['no_base_or_navigation']
    assert not dry['physical_sync_proven']
    argv = next(c for c in dry['components'] if c['name'] == 'frontend')['argv']
    assert Path(argv[argv.index('--params-file')+1]) == config
    tag_path = ROOT/'config/fast_livo2/apriltag_camera.json'
    assert hashlib.sha256(tag_path.read_bytes()).hexdigest() == meta['source_sha256'][str(tag_path.relative_to(ROOT))]
    tag = json.loads(tag_path.read_text())
    assert tag['offset_ns'] == 0 and not tag['time_verified']
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == meta['head']
    assert not (ROOT/'bags/fast-livo-motion-NEW').exists()
    for key, expected in [('turn', '23b3bdba0c4ac5497732eaae61f256bef74d57c52f0181efe58b7f5fe7a48e81'),
                          ('straight', 'e6c7cd1b2d200b4629abfa7002458ace896e3c6a3b2268cc673145d21f400b6d')]:
        result = json.loads((HERE/(key+'_timing.json')).read_text())
        assert result['db_sha256'] == expected
    assert 'Ran 6 tests' in (HERE/'timing_tests.log').read_text()
    assert 'Ran 5 tests' in (HERE/'association_tests.log').read_text()
    for script in [HERE/'investigate.py', Path(__file__)]:
        ast.parse(script.read_text())
    changes = ['AGENTS.md', str(config.relative_to(ROOT)), 'docs/fast_livo2_timestamp_phase_report.md',
               'docs/fast_livo2_interfaces.md', 'docs/fast_livo2_recording.md', 'docs/STATUS.md',
               'docs/HANDOFF.md', 'docs/README.md', 'docs/FAST_LIVO2_APRILTAG_PLAN.md']
    check = {'only_parameter_semantic_change': 'time_offset.img_time_offset: 0.0 -> 0.1',
             'ros_parameter_node_value_s': 0.1, 'ros_domain_id': 211, 'localhost_only': True,
             'parameter_node_destroyed': True, 'other_parameters_unchanged': True,
             'default_record_entry_uses_updated_file': True, 'tag_camera_config_unchanged': True,
             'tag_time_verified': False, 'physical_time_verified': False,
             'raw_full_hashes_match_previous_evidence': True, 'existing_regression_tests': 11,
             'head_unchanged': True, 'no_new_capture_directory': True, 'backup': str(backup),
             'source_sha256': {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in changes}}
    (HERE/'configuration_check.json').write_text(json.dumps(check, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: v for k, v in check.items() if k != 'source_sha256'}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
