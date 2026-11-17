#!/usr/bin/env python3
"""Independent, offline map-connectivity diagnostic. No ROS, devices or commands.

A rectangle contains a circle of radius half its smaller dimension. If an
optimistic superset of that circle's free center space is disconnected, no
continuous rectangular-body path can connect the endpoints, at any heading.
"""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt, label

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'ws_nav/src/ranger_navigation')]
from fast_livo2.navigation.asset import load_map, sha
from ranger_navigation.geometry import load_geometry
from ranger_navigation.models import Limits, Mode
from ranger_navigation.planner import Grid
from prepare_exact_field_test import check_leg


def center_components(blocked, center_distance, resolution, origin, endpoints, radius):
    """Return an optimistic 8-connected superset, not a navigation free mask.

    Let delta=sqrt(2)*resolution/2. Every continuous free center p lies in a
    fine pixel whose representative q is at most delta away. Distance to the
    nearest forbidden pixel CENTER is >= distance to forbidden AREA, so
    D_center(q) >= radius-delta for any valid p. Using radius-2*delta admits
    even more pixels. Thus disconnection cannot result from this allowance
    making the body too large. The 8-neighbor graph also admits corner cuts.
    """
    delta = math.sqrt(2)*resolution/2
    threshold = max(0., radius-2*delta)
    admitted = (~blocked) & (center_distance >= threshold)
    components, _ = label(admitted, np.ones((3, 3), dtype=np.int8))
    indices = np.floor((np.asarray(endpoints)-np.asarray(origin[:2]))/resolution+1e-9).astype(int)
    h, w = blocked.shape
    ids = [int(components[y, x]) if 0 <= x < w and 0 <= y < h else 0 for x, y in indices]
    result = {'circle_radius_m': radius, 'fine_resolution_m': resolution,
              'optimistic_distance_threshold_m': threshold, 'allowance_m': 2*delta,
              'start_component': ids[0], 'goal_component': ids[1],
              'start_admitted': bool(ids[0]), 'goal_admitted': bool(ids[1]),
              'connected': bool(ids[0] and ids[0] == ids[1]),
              'interpretation': 'connected is necessary only; disconnected with admitted endpoints rejects the rectangle too'}
    return result, components


def render(output, grid, components, result, points, body):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font = font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': font.get_name(), 'axes.unicode_minus': False})
    box = [-.5, 4.8, -4.5, 1.5]
    fine = result['fine_resolution_m']
    lo = np.floor((np.array([box[0], box[2]])-grid.origin[:2])/fine).astype(int)
    hi = np.ceil((np.array([box[1], box[3]])-grid.origin[:2])/fine).astype(int)
    crop = components[lo[1]:hi[1], lo[0]:hi[0]]
    a, b = crop == result['start_component'], crop == result['goal_component']
    if not a.any() or not b.any():
        raise ValueError('local review crop does not contain both components')
    dist, nearest = distance_transform_edt(~a, return_indices=True)
    by, bx = np.unravel_index(np.argmin(np.where(b, dist, np.inf)), b.shape)
    ay, ax = nearest[:, by, bx]
    to_world = lambda x, y: grid.origin[:2]+(np.array([x+lo[0], y+lo[1]])+.5)*fine
    pa, pb = to_world(ax, ay), to_world(bx, by)
    midpoint = (pa+pb)/2
    result['local_component_closest_centers_m'] = [pa.tolist(), pb.tolist()]
    result['local_review_center_map_xy'] = midpoint.tolist()
    result['note'] = 'Review center is between optimistic center-space components; not a measured doorway width.'
    rgb = np.full((*grid.cells.shape, 3), 195, dtype=np.uint8)
    rgb[grid.cells == 0] = 255; rgb[grid.cells >= 50] = 15
    extent = [grid.origin[0], grid.origin[0]+grid.cells.shape[1]*grid.resolution,
              grid.origin[1], grid.origin[1]+grid.cells.shape[0]*grid.resolution]
    fine_extent = [grid.origin[0]+lo[0]*fine, grid.origin[0]+hi[0]*fine,
                   grid.origin[1]+lo[1]*fine, grid.origin[1]+hi[1]*fine]
    overlay = np.zeros((*crop.shape, 4))
    overlay[a] = [.05, .45, .95, .40]; overlay[b] = [.95, .42, .05, .40]
    fig, axes = plt.subplots(1, 2, figsize=(13, 8))
    for ax in axes:
        ax.imshow(rgb, origin='lower', extent=extent, interpolation='nearest')
        ax.set_aspect('equal'); ax.set_xlabel('map x / m'); ax.set_ylabel('map y / m')
        ax.set(xlim=box[:2], ylim=box[2:])
        for (x, y), name in zip(points, ['办公室候选位', '门外候选位']):
            ax.scatter(x, y, color='#176fc6', s=28); ax.annotate(name, (x, y), xytext=(5, 8), textcoords='offset points')
        ax.add_patch(plt.Circle(midpoint, .36, fill=False, color='#d81732', linewidth=2))
    axes[0].set_title('原始占据图：黑=占用，灰=未知，白=自由')
    axes[1].imshow(overlay, origin='lower', extent=fine_extent, interpolation='nearest')
    axes[1].plot(*np.vstack((pa, pb)).T, color='#dc1236', linestyle='--', linewidth=2)
    axes[1].set_title('52cm内接圆的乐观中心可行域：蓝/橙不连通')
    fig.suptitle('0.74×0.52m车体；仅检查旧地图，不代表现场门洞测量\n'
                 '圆的条件比矩形宽松，且额外放宽1.41cm仍不连通；局部实时参数保持原值', fontsize=13)
    fig.tight_layout(); fig.savefig(output/'doorway_connectivity.png', dpi=150); plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--navigation-map', type=Path, default=ROOT/'outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--search', action='store_true', help='also run both headings, 3 modes, 5s plus DRIVE/30s searches')
    args = p.parse_args()
    manifest = load_map(args.navigation_map)
    body, _, _ = load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml', .03)
    grid = Grid.from_yaml(args.navigation_map/'map.yaml')
    args.output.mkdir(parents=True, exist_ok=False)
    endpoints = [[.6, 0.], [3.2, -3.2]]
    scale = 5
    blocked = np.repeat(np.repeat(grid.blocked, scale, axis=0), scale, axis=1)
    fine = grid.resolution/scale
    distance = distance_transform_edt(~blocked)*fine
    report = {'kind': 'offline_static_margin_diagnosis', 'hardware_started': False,
              'physical_commands_published': False, 'runtime_parameters_modified': False,
              'body_length_width_m': [body.length, body.width], 'endpoints_map_xy': endpoints,
              'requested_global_static_total_margin_m': .015, 'local_static_total_margin_m': .055,
              'point_voxel_and_coverage_parameters': 'unchanged; no realtime inputs used in this static-only diagnostic',
              'map_uuid': manifest['map_uuid'], 'map_image_sha256': manifest['map_image_sha256'],
              'map_yaml_sha256': manifest['map_yaml_sha256'], 'source_manifest_sha256': manifest['source_manifest_sha256'],
              'script_sha256': sha(__file__), 'connectivity': [], 'searches': []}
    for name, radius in [('point', 0.), ('smaller_circle', .24), ('physical_body_inscribed_circle', min(body.half)),
                          ('requested_margin_circle', min(body.half)+.015), ('local_margin_circle', min(body.half)+.055)]:
        result, components = center_components(blocked, distance, fine, grid.origin, endpoints, float(radius))
        result['name'] = name
        if name == 'physical_body_inscribed_circle':
            render(args.output, grid, components, result, endpoints, body)
        report['connectivity'].append(result)
        print(json.dumps(result), flush=True)
    if args.search:
        for budget, mode in [(5., m) for m in Mode]+[(30., Mode.DRIVE)]:
            for name, start, goal in [('exit', [.6,0.,0.], [3.2,-3.2,-math.pi/2]),
                                       ('return', [3.2,-3.2,math.pi/2], [.6,0.,0.])]:
                # No dynamic points or local visibility participate here.
                # Removing physical/engineering inflation only for the static
                # planner leaves its 15mm swept-motion bound, with the exact
                # same 74x52cm rectangle. Runtime body/parameters stay intact.
                trial = check_leg(grid, replace(body, clearance=0.), Limits(speed=.1,yaw_rate=.2),
                                  start, goal, mode, budget, static_reserve=0., prefer_parallel_in_narrow=True)
                trial['name'] = name; report['searches'].append(trial)
                print(json.dumps({k:v for k,v in trial.items() if k not in ('path','segments')}), flush=True)
    (args.output/'diagnosis.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
