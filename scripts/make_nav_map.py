#!/usr/bin/env python3
"""生成导航用地图副本（不清改已验收原图）。

背景：建图起点 (0,0) 附近存在距中心 0.177 m 的占据像元——建图时机器人本体
就覆盖该位置，物理上不可能是真实障碍（近体/地面泄漏点），但经 costmap 膨胀
（内切半径环）会把起点判为致死，Smac 拒绝规划（"Starting point in lethal
space"，证据 docs/06_evidence/safety-15ege02f）。本脚本在副本中清除起点
--radius 米内的占据像元；原图 map.pgm/map.yaml 保持不变。

用法:
  python3 scripts/make_nav_map.py                       # 默认起点(0,0) r=0.30
  python3 scripts/make_nav_map.py --radius 0.25
输出: <map_dir>/nav/map_nav.pgm + map_nav.yaml
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--map-yaml',
               default='maps/lightning-offline-suej7qsz/data/new_map/map.yaml')
p.add_argument('--radius', type=float, default=0.30,
               help='清除半径（米）；必须 < footprint 最小半边 0.26+padding 的物理占据范围')
p.add_argument('--center', nargs=2, type=float, default=[0.0, 0.0],
               help='建图起点（map 系，米）')
a = p.parse_args()

src = Path(a.map_yaml).resolve()
d = src.parent
y = yaml.safe_load(open(src))
img = np.array(Image.open(d / y['image']))
H, W = img.shape
res = y['resolution']
ox, oy = y['origin'][0], y['origin'][1]
cx, cy = a.center
half_min = 0.26  # footprint 半宽（0.52/2）；清除半径不得超过
assert a.radius <= half_min + 0.05, f'radius {a.radius} 过大，可能清除真实障碍'

out = img.copy()
cleared = []
for r, c in zip(*np.nonzero(img < 127)):          # occupied (trinary 暗像元)
    wx = ox + (c + 0.5) * res
    wy = oy + (H - 1 - r + 0.5) * res
    if np.hypot(wx - cx, wy - cy) <= a.radius:
        out[r, c] = 255
        cleared.append((round(wx, 2), round(wy, 2)))

nd = d / 'nav'
nd.mkdir(exist_ok=True)
Image.fromarray(out).save(nd / 'map_nav.pgm')
y2 = dict(y)
y2['image'] = 'map_nav.pgm'
(nd / 'map_nav.yaml').write_text(yaml.safe_dump(y2, sort_keys=False))
print(f'cleared {len(cleared)} occupied cells within {a.radius} m of ({cx},{cy}): {cleared}')
print(f'wrote {nd}/map_nav.pgm ({W}x{H}) and {nd}/map_nav.yaml（原图未改动）')
