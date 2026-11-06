#!/usr/bin/env python3
"""Export every paired IMU interval once, using the final B graph when supplied.

No ROS node, sensor, TF or registration is started. Native messages stay read-only.
The overlay PCD adds display markers; map_full.pcd contains sensor geometry only.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

from validate_backend import matrix


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(8*1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def pcd_open(path, count, rgb=False):
    stream = path.open('xb')
    stream.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z '+('rgb' if rgb else 'intensity')+
                  '\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH '+str(count)+
                  '\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS '+str(count)+'\nDATA binary\n').encode())
    return stream


def body_points(message):
    assert message.header.frame_id == 'fast_livo_imu'
    assert not message.is_bigendian and len(message.data) == message.row_step*message.height
    assert message.row_step >= message.point_step*message.width
    fields = {field.name: field for field in message.fields}
    columns = []
    for name in ('x', 'y', 'z', 'intensity'):
        field = fields[name]
        assert field.datatype == PointField.FLOAT32 and field.count == 1
        assert field.offset+4 <= message.point_step
        columns.append(np.ndarray((message.height, message.width), dtype='<f4', buffer=message.data,
                                  offset=field.offset, strides=(message.row_step, message.point_step)).reshape(-1))
    points = np.column_stack(columns)
    assert np.isfinite(points).all()
    return points


def transform(points, pose):
    result = points.copy()
    result[:, :3] = points[:, :3].astype(float) @ pose[:3, :3].T+pose[:3, 3]
    assert np.isfinite(result).all()
    return result


def voxel(points, resolution):
    _, indices = np.unique(np.floor(points/resolution).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(indices)]


def markers(positions):
    pieces = []
    for left, right in zip(positions, positions[1:]):
        steps = max(1, int(np.ceil(np.linalg.norm(right-left)/.025)))
        pieces.append(left+np.arange(steps)[:, None]/steps*(right-left))
    line = voxel(np.vstack([*pieces, positions[-1:]]), .015)
    offsets = np.vstack([np.zeros(3), np.eye(3)*.035, -np.eye(3)*.035])
    tube = (line[:, None, :]+offsets[None, :, :]).reshape(-1, 3).astype('<f4')
    angles = np.linspace(0, 2*np.pi, 80, endpoint=False)
    circles = np.column_stack([.13*np.cos(angles), .13*np.sin(angles), np.zeros(len(angles))])
    points = np.vstack([tube, positions[0]+circles, positions[-1]+circles]).astype('<f4')
    colors = np.r_[np.full(len(tube), 0xf43f33, dtype='<u4'),
                   np.full(len(circles), 0x10b981, dtype='<u4'),
                   np.full(len(circles), 0xb123d1, dtype='<u4')]
    return points, colors


def plot(points, positions, times, output, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': font.get_name(), 'axes.unicode_minus': False,
                         'font.size': 11, 'savefig.facecolor': '#ffffff'})
    relative = (times-times[0])*1e-9

    def path_2d(axis):
        axis.plot(positions[:, 0], positions[:, 1], color='white', lw=3.6, zorder=5)
        axis.plot(positions[:, 0], positions[:, 1], color='#d92f31', lw=1.8, label='IMU轨迹', zorder=6)
        axis.scatter(*positions[0, :2], c='#059669', s=95, marker='o', edgecolors='white', zorder=8)
        axis.scatter(*positions[-1, :2], c='#aa22bc', s=100, marker='X', edgecolors='white', zorder=9)
        for index, text, delta, color in [(0, '起点', (12, 15), '#047857'), (-1, '终点', (12, -25), '#9d1bad')]:
            axis.annotate(text, positions[index, :2], xytext=delta, textcoords='offset points', color=color,
                          weight='bold', bbox={'fc': 'white', 'ec': color, 'alpha': .9, 'boxstyle': 'round,pad=.25'},
                          arrowprops={'arrowstyle': '-', 'color': color}, zorder=12)
        for number, seconds in enumerate(np.arange(60, relative[-1], 60)):
            index = min(int(np.searchsorted(relative, seconds)), len(positions)-1)
            axis.scatter(*positions[index, :2], s=16, c='#d92f31', zorder=7)
            axis.annotate(f'{int(seconds)}s', positions[index, :2], xytext=(10, 8 if number % 2 else -15),
                          textcoords='offset points', fontsize=9, color='#9e1823',
                          bbox={'fc': 'white', 'ec': 'none', 'alpha': .88, 'pad': 1}, zorder=10)
        axis.set_aspect('equal', adjustable='box')
        axis.set(xlabel='地图 X / m', ylabel='地图 Y / m')
        axis.grid(alpha=.16)

    draw = points[::max(1, int(np.ceil(len(points)/450000)))]
    zlo, zhi = np.percentile(points[:, 2], [1, 99])
    figure = plt.figure(figsize=(19, 10), layout='constrained')
    left = figure.add_subplot(1, 2, 1)
    left.scatter(draw[:, 0], draw[:, 1], c=draw[:, 2], cmap='bone_r', vmin=zlo, vmax=zhi,
                 s=.3, alpha=.55, linewidths=0, rasterized=True)
    path_2d(left)
    left.set_title('完整点云俯视 · 全空间范围', weight='bold', pad=12)
    right = figure.add_subplot(1, 2, 2, projection='3d', computed_zorder=False)
    sparse = draw[::max(1, int(np.ceil(len(draw)/220000)))]
    right.scatter(sparse[:, 0], sparse[:, 1], sparse[:, 2], c=sparse[:, 2], cmap='viridis',
                  vmin=zlo, vmax=zhi, s=.25, alpha=.4, linewidths=0, rasterized=True, zorder=1)
    right.plot(*positions.T, c='#e83431', lw=1.6, zorder=5)
    right.scatter(*positions[0], c='#059669', s=55, zorder=6)
    right.scatter(*positions[-1], c='#aa22bc', marker='X', s=65, zorder=7)
    right.set(xlabel='X / m', ylabel='Y / m', zlabel='Z / m', title='完整点云三维视图 · 红色为轨迹')
    right.set_box_aspect(np.maximum(np.ptp(points, axis=0), .2))
    right.view_init(elev=48, azim=-65)
    figure.suptitle('走廊 / 教室 · 全包建图与轨迹', fontsize=20, weight='bold')
    figure.supxlabel(f"+100 ms；{summary['kind_label']}；{summary['paired_frames']:,} 对输入；"
                     f"{summary['full_map_points']:,} 点；轨迹约 {summary['estimated_path_length_m']:.1f} m（估计值）\n"
                     'PCD保留全部有效区间点；图片仅降采样显示；Tag关闭；无独立真值', fontsize=11)
    figure.savefig(output/'map_overview.png', dpi=180)
    plt.close(figure)

    floor_reference = float(np.median(positions[:, 2]))
    low, high = floor_reference+.05, floor_reference+1.8
    keep = (points[:, 2] >= low) & (points[:, 2] <= high)
    wall = points[keep]
    if len(wall):
        figure, axis = plt.subplots(figsize=(12, 12), layout='constrained')
        axis.scatter(wall[:, 0], wall[:, 1], c='#4d5967', s=.35, alpha=.65, linewidths=0, rasterized=True)
        path_2d(axis)
        axis.set_xlim(positions[:, 0].min()-5, positions[:, 0].max()+5)
        axis.set_ylim(positions[:, 1].min()-5, positions[:, 1].max()+5)
        axis.set_title(f'轨迹周边细看 · 墙体高度显示 [{low:.2f}, {high:.2f}] m\n完整PCD没有裁剪', fontsize=15, pad=14)
        figure.supxlabel('显示范围为轨迹外扩5m；时间标注相对首个有效三融合输出；起终点接近时标签错开显示', fontsize=10)
        figure.savefig(output/'map_detail.png', dpi=190)
        plt.close(figure)
    return {'display_voxel_m': .08, 'display_points': len(points),
            'height_slice_m': [low, high], 'detail_xy_margin_m': 5.}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--native', type=Path, required=True)
    parser.add_argument('--frames', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path)
    parser.add_argument('--mode', choices=['frontend', 'backend_diagnostic'], default='frontend')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print('DRY-RUN: export all paired native intervals and final-graph trajectory')
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = None
    keyframes = []
    if args.mode == 'backend_diagnostic':
        if args.snapshot is None:
            parser.error('backend_diagnostic requires --snapshot')
        manifest = json.loads((args.snapshot/'manifest.json').read_text())
        committed = json.loads((args.snapshot/'COMMITTED').read_text())
        assert committed['manifest_sha256'] == sha(args.snapshot/'manifest.json')
        assert manifest['provenance']['frontend_gravity_aligned'] is True
        keyframes = manifest['keyframes']
        key_indices = np.array([k['scan_id'] for k in keyframes], dtype=np.int64)
        assert key_indices[0] == 0 and np.all(np.diff(key_indices) > 0)
        corrections = [matrix(k['X_map_imu']) @ np.linalg.inv(matrix(k['U_local_imu'])) for k in keyframes]
    connection = sqlite3.connect(next(args.native.glob('*.db3')).resolve().as_uri()+'?mode=ro', uri=True)
    connection.execute('PRAGMA query_only=ON')
    ids = {name: identifier for identifier, name in connection.execute('SELECT id,name FROM topics')}
    poses = {}
    for data, in connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (ids['/fast_livo/odometry'],)):
        message = deserialize_message(data, Odometry)
        assert message.header.frame_id == 'fast_livo_local' and message.child_frame_id == 'fast_livo_imu'
        assert stamp(message) not in poses
        poses[stamp(message)] = matrix(message.pose.pose)
    times = np.array(sorted(poses), dtype=np.int64)
    assert np.all(np.diff(times) > 0)
    raw = [poses[int(t)] for t in times]
    final = []
    for index, pose in enumerate(raw):
        if manifest is None:
            final.append(pose)
        else:
            anchor = int(np.searchsorted(key_indices, index, side='right')-1)
            final.append(corrections[anchor] @ pose)
    for key in keyframes:
        assert key['stamp_ns'] == int(times[key['scan_id']])
        assert np.max(np.abs(final[key['scan_id']]-matrix(key['X_map_imu']))) < 1e-8
    positions = np.array([pose[:3, 3] for pose in final])
    display_markers, marker_colors = markers(positions)
    with args.frames.open() as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == len(times)
    total = sum(int(row['points']) for row in rows)
    samples = []
    lower = np.full(3, np.inf)
    upper = np.full(3, -np.inf)
    written = 0
    with pcd_open(args.output/'map_full.pcd', total) as optimized, \
         pcd_open(args.output/'frontend_map_full.pcd', total) as frontend, \
         pcd_open(args.output/'map_with_trajectory.pcd', total+len(display_markers), rgb=True) as overlay:
        cloud_count = 0
        for index, (data,) in enumerate(connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (ids['/fast_livo/deskewed_imu'],))):
            cloud = deserialize_message(data, PointCloud2)
            assert index < len(times) and stamp(cloud) == int(times[index])
            body = body_points(cloud)
            assert len(body) == int(rows[index]['points'])
            local = transform(body, raw[index])
            mapped = transform(body, final[index])
            frontend.write(np.asarray(local, dtype='<f4').tobytes())
            optimized.write(np.asarray(mapped, dtype='<f4').tobytes())
            colored = np.asarray(mapped, dtype='<f4').copy()
            colored[:, 3].view('<u4')[:] = 0x7c8793
            overlay.write(colored.tobytes())
            lower = np.minimum(lower, mapped[:, :3].min(axis=0))
            upper = np.maximum(upper, mapped[:, :3].max(axis=0))
            samples.append(mapped[index % 8::8, :3].copy())
            written += len(mapped)
            cloud_count += 1
            if index % 500 == 0:
                print(json.dumps({'exported_frames': index+1, 'points': written}), flush=True)
        assert cloud_count == len(times) and written == total
        decorated = np.zeros((len(display_markers), 4), dtype='<f4')
        decorated[:, :3] = display_markers
        decorated[:, 3].view('<u4')[:] = marker_colors
        overlay.write(decorated.tobytes())
    connection.close()
    relative = (times-times[0])*1e-9
    with (args.output/'trajectory_imu.tum').open('x') as tum, (args.output/'trajectory_imu.csv').open('x', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['stamp_ns', 'relative_seconds', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for stamp_ns, seconds, pose in zip(times, relative, final):
            values = [*pose[:3, 3], *Rotation.from_matrix(pose[:3, :3]).as_quat()]
            writer.writerow([int(stamp_ns), f'{seconds:.9f}', *values])
            tum.write(f'{stamp_ns//10**9}.{stamp_ns%10**9:09d} '+' '.join(f'{value:.12f}' for value in values)+'\n')
    with pcd_open(args.output/'trajectory.pcd', len(times)) as stream:
        stream.write(np.column_stack([positions, relative]).astype('<f4').tobytes())
    for name, count in [('map_full.pcd', total), ('frontend_map_full.pcd', total),
                        ('map_with_trajectory.pcd', total+len(display_markers)), ('trajectory.pcd', len(times))]:
        with (args.output/name).open('rb') as stream:
            while True:
                line = stream.readline()
                assert line, 'missing PCD DATA header: '+name
                if line.strip() == b'DATA binary':
                    break
            assert (args.output/name).stat().st_size-stream.tell() == count*16
    summary = {'paired_frames': len(times), 'full_map_points': total, 'display_marker_points': len(display_markers),
               'mode': args.mode, 'kind_label': '前端三融合 · 未应用回环优化' if manifest is None else '后端诊断图 · 验收未通过',
               'map_uuid': manifest['map_uuid'] if manifest else None,
               'graph_version': manifest['graph_version'] if manifest else None,
               'keyframes': len(keyframes), 'applied_loop_factors': sum(k['loop'] for k in manifest['factors']) if manifest else 0,
               'input_frame': 'fast_livo_imu', 'output_frame': 'map' if manifest else 'fast_livo_local', 'transform_convention': 'T_A_B maps B to A',
               'point_transform': 'p_map = (X_k_final * inverse(U_k) * U_t) * p_imu; preceding keyframe k' if manifest else 'p_local = U_t * p_imu',
               'frontend_point_transform': 'p_local = U_t * p_imu',
               'lidar_imu_extrinsic_reapplied': False, 'per_interval_cloud_used_once': True,
               'estimated_path_length_m': float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
               'endpoint_separation_m': float(np.linalg.norm(positions[-1]-positions[0])),
               'source_duration_s': float(relative[-1]), 'bounds_m': [lower.tolist(), upper.tolist()],
               'height_aligned_frontend': True, 'tag_used': False, 'independent_accuracy_validated': False,
               'source_manifest_sha256': committed['manifest_sha256'] if manifest else None,
               'overlay_is_for_visualization_only': True}
    sample = voxel(np.vstack(samples), .08)
    summary['display'] = plot(sample, positions, times, args.output, summary)
    summary['files_sha256'] = {p.name: sha(p) for p in sorted(args.output.iterdir()) if p.is_file()}
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
