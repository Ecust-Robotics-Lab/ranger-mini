"""Read-only startup diagnosis from the owned camera, LiDAR and health logs."""
import json
import re


def log_tail(path):
    try:
        with path.open('rb') as handle:
            handle.seek(max(0, path.stat().st_size - 16_000))
            return handle.read().decode(errors='replace')
    except OSError:
        return ''


def camera_startup_detail(run, components):
    """Report missing frames separately from a camera that never opened."""
    if components.get('camera', {}).get('state') != 'STARTING':
        return None
    try:
        health = json.loads((run / 'health.json').read_text())
    except (OSError, ValueError):
        return None
    checks = health.get('checks', {})
    if checks.get('camera') or not (checks.get('lidar') and checks.get('writer')):
        return None

    camera = log_tail(run / 'camera.log')
    if 'Camera connected and grabbing started' not in camera:
        return None
    if 'Publishing image:' in camera or health.get('counts', {}).get('camera', 0):
        return '相机已有出图记录，当前没有新鲜同步图像。请检查相机同步状态。'
    if 'GetImageBuffer timeout: no frame' not in camera:
        return '相机已打开，正在等待首帧。'

    lidar = log_tail(run / 'lidar.log')
    time_types = re.findall(r'STAGE3_TIME .*?\btime_type=(\d+)', lidar)
    reason = '相机已打开，但 SDK 等待外触发图像超时（MV_E_NODATA）。'
    if time_types and time_types[-1] == '0':
        return reason + ' 雷达当前未同步（time_type=0），请检查外部同步与相机触发信号。'
    return reason + ' 请检查相机外触发信号与连接。'
