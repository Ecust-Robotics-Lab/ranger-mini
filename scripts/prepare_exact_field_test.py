#!/usr/bin/env python3
"""Offline field preparation: inspect immutable map, plan candidate legs, render review.

Never imports ROS or opens CAN/sensors. Output is a new directory; a successful
process exit means preparation completed, not that the field route is accepted.
"""
import argparse
import base64
import io
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'ws_nav/src/ranger_navigation'), str(ROOT/'scripts')]
from ranger_navigation.geometry import load_geometry, validate_sweep, wrap
from ranger_navigation.models import Limits, Mode
from ranger_navigation.planner import Grid, Planner, NoPath
from fast_livo2.navigation.asset import load_map, sha

DEFAULT_MAP = ROOT/'outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final'


def read_route(path, manifest):
    data = yaml.safe_load(Path(path).read_text())
    if (data.get('schema_version') != 1 or data.get('frame') != 'map'
            or data.get('map_uuid') != manifest['map_uuid']
            or data.get('source_manifest_sha256') != manifest['source_manifest_sha256']):
        raise ValueError('route schema/frame/map identity mismatch')
    points = data.get('waypoints', [])
    if not 2 <= len(points) <= 64:
        raise ValueError('route requires 2..64 waypoints')
    names = set()
    for p in points:
        pose = np.asarray(p['pose'], dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError('waypoint must contain finite x/y/yaw')
        if not isinstance(p['name'], str) or not p['name'] or p['name'] in names:
            raise ValueError('waypoint names must be unique nonempty strings')
        names.add(p['name'])
        p['pose'] = [float(pose[0]), float(pose[1]), float(wrap(pose[2]))]
    a, b = np.asarray(points[0]['pose']), np.asarray(points[-1]['pose'])
    if np.linalg.norm(a[:2]-b[:2]) > .15 or abs(wrap(a[2]-b[2])) > .15:
        raise ValueError('return route must end at the departure pose within .15m/.15rad')
    return data


def check_leg(grid, body, limits, start, goal, start_mode=Mode.DRIVE, budget=5., static_reserve=.04, prefer_parallel_in_narrow=False):
    planner = Planner(body=body, limits=limits, timeout=budget, static_reserve=static_reserve,
                      prefer_parallel_in_narrow=prefer_parallel_in_narrow)
    began = time.monotonic()
    report = {'start': list(start), 'goal': list(goal), 'budget_s': budget,
              'start_mode': int(start_mode), 'accepted': False, 'path': [], 'segments': []}
    try:
        segments = planner.plan(grid, start, goal, start_mode=start_mode)
        path = np.vstack([s.poses for s in segments]) if segments else np.array([start, goal])
        okay, _, reason = validate_sweep(path, np.empty((0, 2)), body, grid=grid,
                                         margin=body.clearance+.04,
                                         map_margin=body.clearance+static_reserve)
        if not okay:
            raise NoPath('independent_validation:'+reason)
        report.update(accepted=True, reason='static_map_swept_body_validated', path=path.tolist(),
                      length_m=float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum()),
                      segments=[{'mode': s.mode.name, 'reverse': s.reverse,
                                 'poses': s.poses.tolist()} for s in segments])
    except NoPath as exc:
        report['reason'] = str(exc)
    report.update(elapsed_s=time.monotonic()-began, expansions=planner.expansions)
    return report


def read_only_command(argv):
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=10)
        return {'argv': argv, 'exit_code': r.returncode, 'stdout': r.stdout, 'stderr': r.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'argv': argv, 'exit_code': None, 'error': str(exc)}


def current_identity():
    commands = [['git', 'rev-parse', 'HEAD'], ['git', 'status', '--short'],
                ['ip', '-json', '-details', 'link', 'show', 'can0'],
                ['nvidia-smi', '--query-gpu=name,driver_version,memory.total', '--format=csv,noheader']]
    # Read /proc only; no DDS discovery, no connection to any device.
    processes = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            argv = (p/'cmdline').read_bytes().split(b'\0')
            name = Path(argv[0].decode()).name
            command = [s.decode(errors='replace') for s in argv if s]
            if (name in ('ranger_base_node', 'localizer_node', 'fastlivo_mapping',
                         'hik_camera_trigger_driver_node', 'livox_ros_driver2_node')
                    or any(Path(s).name in ('exact_navigation', 'motion_gate', 'worker.py') for s in command[:3])):
                processes.append({'pid': int(p.name), 'argv': command,
                                  'start_ticks': (p/'stat').read_text().split(') ', 1)[1].split()[19]})
        except (OSError, IndexError, UnicodeError):
            pass
    return {'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'free_bytes': shutil.disk_usage(ROOT).free, 'observed_processes': processes,
            'read_only_commands': [read_only_command(a) for a in commands]}


def render(output, report, route, manifest, grid, body):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font = font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': font.get_name(), 'axes.unicode_minus': False})
    rgb = np.full((*grid.cells.shape, 3), 202, dtype=np.uint8)
    rgb[grid.cells == 0] = 255
    rgb[grid.cells >= 50] = 18
    png = io.BytesIO()
    Image.fromarray(np.flipud(rgb)).save(png, format='PNG')
    image_url = 'data:image/png;base64,'+base64.b64encode(png.getvalue()).decode()
    extent = [grid.origin[0], grid.origin[0]+grid.cells.shape[1]*grid.resolution,
              grid.origin[1], grid.origin[1]+grid.cells.shape[0]*grid.resolution]
    xy = np.asarray([p['pose'][:2] for p in route['waypoints']])
    def bounds(points, padding):
        low, high = points.min(0)-padding, points.max(0)+padding
        return [float(low[0]), float(high[0]), float(low[1]), float(high[1])]
    turn = next((p for p in route['waypoints'] if p['name']=='turn_position'), route['waypoints'][len(xy)//2])
    views = {'overview': bounds(xy, 1.5), 'door': bounds(xy[:2], 1.),
             'end': bounds(np.asarray([turn['pose'][:2]]), 2.)}
    wide = np.ptp(xy[:,0])>1.3*np.ptp(xy[:,1])
    fig, axes = plt.subplots(2, 1, figsize=(13, 10)) if wide else plt.subplots(1, 2, figsize=(13, 10))
    for ax in axes:
        ax.imshow(rgb, origin='lower', extent=extent)
        for leg in report['legs']:
            path = np.array(leg['path'] if leg['accepted'] else [leg['start'], leg['goal']])
            ax.plot(*path[:, :2].T, color='#07844b' if leg['accepted'] else '#e34b43',
                    linestyle='-' if leg['accepted'] else '--', linewidth=1.8)
        for i, p in enumerate(route['waypoints']):
            x, y, yaw = p['pose']
            ax.scatter(x, y, s=30, color='#2563eb')
            ax.annotate(str(i), (x, y), xytext=(6, 4), textcoords='offset points', color='#183fa1')
            c, s = math.cos(yaw), math.sin(yaw)
            poly = body.footprint@np.array([[c, s], [-s, c]])+np.array([x, y])
            ax.fill(*poly.T, color='#268cdb', alpha=.25)
        ax.set_aspect('equal'); ax.set_xlabel('map x / m'); ax.set_ylabel('map y / m')
        ax.grid(alpha=.12)
    axes[0].set(xlim=views['overview'][:2], ylim=views['overview'][2:], title='办公室 → 走廊端点 → 返回（候选）')
    axes[1].set(xlim=views['door'][:2], ylim=views['door'][2:], title='办公室与门口放大：红虚线不是可执行路径')
    fig.suptitle('仅离线静态地图检查；绿色为独立扫掠检查通过，红色为未找到路线\n'
                 '点位不是当前实车位姿，现场门口/障碍/定位/制动均待验', fontsize=13)
    fig.tight_layout(); fig.savefig(output/'route_review.png', dpi=150); plt.close(fig)
    data = {'report': report, 'route': route, 'manifest': manifest, 'image': image_url,
            'extent': extent, 'views': views, 'wide': bool(wide), 'body': [body.length, body.width]}
    template = (ROOT/'scripts/exact_field_route_review.html').read_text()
    # Escape script terminators, including in externally supplied waypoint labels.
    payload = json.dumps(data, ensure_ascii=False).replace('<', '\\u003c')
    (output/'index.html').write_text(template.replace('__FIELD_DATA__', payload))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--navigation-map', type=Path, default=DEFAULT_MAP)
    p.add_argument('--route', type=Path, default=ROOT/'config/exact_mppi/office_corridor_route.yaml')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--clearance-profile', choices=('standard', 'narrow'), default='standard')
    args = p.parse_args()
    manifest = load_map(args.navigation_map)
    route = read_route(args.route, manifest)
    geometry = ROOT/'config/fast_livo2/navigation_prep.yaml'
    config = ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml'
    cfg = yaml.safe_load(config.read_text())
    profile = cfg['clearance_profiles'][args.clearance_profile]
    body, _, _ = load_geometry(geometry, float(profile['clearance']))
    low = cfg['real_vehicle_first_test']
    limits = Limits(speed=float(low['max_speed']), yaw_rate=float(low['max_yaw_rate']),
                    acceleration=float(cfg['motion']['acceleration']),
                    yaw_acceleration=float(cfg['motion']['yaw_acceleration']),
                    min_radius=float(cfg['motion']['min_radius']))
    args.output.mkdir(parents=True, exist_ok=False)
    grid = Grid.from_yaml(args.navigation_map/'map.yaml')
    report = {'schema_version': 1, 'kind': 'exact_field_preparation', 'created_unix_s': time.time(),
              'hardware_started': False, 'commands_published': False, 'route': str(args.route.resolve()),
              'map_uuid': manifest['map_uuid'], 'source_manifest_sha256': manifest['source_manifest_sha256'],
              'map_yaml_sha256': manifest['map_yaml_sha256'], 'map_image_sha256': manifest['map_image_sha256'],
              'geometry_sha256': sha(geometry), 'navigation_config_sha256': sha(config),
              'body_length_width_m': [body.length, body.width], 'clearance_m': body.clearance,
              'clearance_profile': args.clearance_profile, 'static_map_reserve_m': profile['static_map_reserve'],
              'human_confirmed': route.get('human_confirmed') is True, 'low_speed_limits': low,
              'machine_snapshot': current_identity(), 'legs': []}
    for a, b in zip(route['waypoints'][:-1], route['waypoints'][1:]):
        # All supported initial feedback modes are checked: later replanning
        # must use the actual mode, not the mode assumed by this review.
        variants = [check_leg(grid, body, limits, a['pose'], b['pose'], mode,
                              static_reserve=profile['static_map_reserve'],
                              prefer_parallel_in_narrow=profile['prefer_parallel_in_narrow']) for mode in Mode]
        accepted = all(v['accepted'] for v in variants)
        representative = next((v for v in variants if not v['accepted']), variants[0]).copy()
        representative.update(start_name=a['name'], goal_name=b['name'], accepted=accepted,
                              mode_checks=[{k: v for k, v in item.items() if k not in ('path', 'segments')}
                                           for item in variants])
        report['legs'].append(representative)
        print(f"{a['name']} -> {b['name']}: {'PASS' if accepted else 'PARTIAL'} {representative['reason']}", flush=True)
    report['all_static_legs_passed'] = all(s['accepted'] for s in report['legs'])
    report['field_route_accepted'] = False
    report['status'] = 'OFFLINE_CANDIDATE_PASS' if report['all_static_legs_passed'] else 'PARTIAL'
    report['limitations'] = ['Candidate departure pose is not a measured live pose.',
        'Search budget exhaustion does not prove geometric impossibility.',
        'Static map validation cannot establish current obstacle clearance or sensing coverage.',
        'No physical direction/mode/braking/localization/navigation validation performed.']
    (args.output/'preparation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    (args.output/'route.json').write_text(json.dumps(route, ensure_ascii=False, indent=2)+'\n')
    render(args.output, report, route, manifest, grid, body)
    identity = {str(f.relative_to(ROOT)): sha(f) for f in [Path(__file__),
        ROOT/'scripts/exact_field_route_review.html', args.route, geometry, config,
        ROOT/'ws_nav/src/ranger_navigation/ranger_navigation/planner.py',
        ROOT/'ws_nav/src/ranger_navigation/ranger_navigation/geometry.py'] if f.is_relative_to(ROOT)}
    (args.output/'sources.json').write_text(json.dumps(identity, indent=2)+'\n')
    print(json.dumps({'status': report['status'], 'output': str(args.output),
                      'hardware_started': False, 'commands_published': False}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
