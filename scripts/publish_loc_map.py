#!/usr/bin/env python3
"""Publish the accepted 2D navigation map (map.pgm + map.yaml) as a latched
nav_msgs/OccupancyGrid on /map for visualization (Foxglove/RViz).

Read-only: loads the map from the accepted experiment directory and publishes
with transient_local durability; touches no hardware, no chassis.
"""
import argparse
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

import yaml
from nav_msgs.msg import OccupancyGrid
from PIL import Image
import numpy as np


def load_grid(map_dir: Path):
    meta = yaml.safe_load((map_dir / "map.yaml").read_text())
    img = np.array(Image.open(map_dir / meta["image"]))
    occ_prob = (255.0 - img.astype(np.float64)) / 255.0
    if meta.get("negate", 0):
        occ_prob = 1.0 - occ_prob
    occ = np.full(img.shape, -1, dtype=np.int8)
    occ[occ_prob < meta["free_thresh"]] = 0
    occ[occ_prob > meta["occupied_thresh"]] = 100
    occ = np.flipud(occ)  # pgm row0 = max y; OccupancyGrid data starts at min y
    return meta, occ


class MapPublisher(Node):

    def __init__(self, map_dir: Path):
        super().__init__("loc_map_publisher")
        meta, occ = load_grid(map_dir)
        self.msg = OccupancyGrid()
        self.msg.header.frame_id = "map"
        self.msg.info.resolution = float(meta["resolution"])
        self.msg.info.width = occ.shape[1]
        self.msg.info.height = occ.shape[0]
        ox, oy, oyaw = meta["origin"]
        self.msg.info.origin.position.x = float(ox)
        self.msg.info.origin.position.y = float(oy)
        self.msg.info.origin.position.z = 0.0
        self.msg.info.origin.orientation.z = math.sin(float(oyaw) / 2.0)
        self.msg.info.origin.orientation.w = math.cos(float(oyaw) / 2.0)
        self.msg.data = occ.reshape(-1).tolist()

        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(OccupancyGrid, "/map", qos)
        # republish at 0.5 Hz in addition to transient_local: harmless, and
        # guarantees late or restarted viewers receive it
        self.timer = self.create_timer(2.0, self.publish)
        self.publish()
        self.get_logger().info(
            f"map published: {self.msg.info.width}x{self.msg.info.height} @ "
            f"{self.msg.info.resolution} m, origin ({ox},{oy})")

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map-dir", type=Path)
    args = ap.parse_args()
    if args.map_dir is None:
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ws_nav/src/ranger_localization_bridge'))
        from ranger_localization_bridge.runtime_config import resolve
        args.map_dir = Path(resolve()['MAP_DIR'])
    rclpy.init()
    node = MapPublisher(args.map_dir.resolve())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
