#!/usr/bin/env python3
"""Stage C1: checkerboard image capture for Hikvision MV-CU013 intrinsics.

Subscribes /front/image_sync (requires the Stage B stack: lidar driver +
timeshare_writer + hik_camera_trigger_driver). Manual save only — no
auto-capture from the 10 Hz stream. Saves raw, unscaled, uncropped,
not geometrically corrected PNGs plus a per-attempt metadata CSV.

Quality gate before a save is accepted:
  - full 11x8 inner corners detected (findChessboardCornersSB)
  - sharpness (Laplacian variance) above threshold
  - brightness in range, overexposed fraction below limit
  - board bounding box fully inside the image (with margin)
  - corners not too similar to any already-saved sample

Modes:
  gui       (default)  live view + SPACE save, q/ESC quit
  headless             terminal status lines, SPACE+Enter or s+Enter save, q quit
"""
import argparse
from pathlib import Path
import csv
import json
from datetime import datetime
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

PATTERN = (11, 8)          # inner corners (cols, rows)
SQUARE = 0.020             # m, recorded in metadata; solver uses this too

# --- quality thresholds (conservative; violations warn/block with reason) ---
BLUR_MIN = 60.0            # Laplacian variance; lower = blurrier
BRIGHT_MIN, BRIGHT_MAX = 25.0, 225.0
OVEREXP_FRAC_MAX = 0.005   # fraction of pixels >= 250
BORDER_MARGIN_PX = 4       # board bbox must keep this margin inside the image
SIMILARITY_PX = 20.0       # min mean corner displacement vs every saved sample


class FrameState:
    """Latest frame + its analysis, guarded by the GIL (single sub callback)."""
    def __init__(self):
        self.img = None            # BGR uint8
        self.received_at = 0.0
        self.detected = False
        self.stamp_ns = 0
        self.ok = False
        self.corners = None        # float32 (N,1,2)
        self.metrics = {}
        self.reason = "no frame yet"


class CaptureNode(Node):
    def __init__(self, args):
        super().__init__("checkerboard_capture")
        self.args = args
        self.state = FrameState()
        self.saved = []            # list of corner arrays of accepted samples
        self.n_saved = 0
        self.n_attempts = 0
        os.makedirs(args.outdir, exist_ok=True)
        images_dir = os.path.join(args.outdir, "images")
        os.makedirs(images_dir, exist_ok=True)
        if any(os.scandir(images_dir)) or os.path.exists(os.path.join(args.outdir, "session_info.json")):
            raise ValueError("images/ is not empty; use a new --outdir to protect existing data")
        self.images_dir = images_dir
        self.resolution = None
        with open(os.path.join(args.outdir, "session_info.json"), "w") as f:
            json.dump(dict(started_at=datetime.now().astimezone().isoformat(),
                           topic=args.topic, pattern=list(PATTERN), square_size_m=SQUARE,
                           distortion_model="plumb_bob", qos=args.qos,
                           timestamp_note="header stamp is publisher time; wall_time_ns is local save time"), f, indent=2)
        self.csv_path = os.path.join(args.outdir, "capture_metadata.csv")
        new = not os.path.exists(self.csv_path)
        self.csv = open(self.csv_path, "a", newline="")
        self.writer = csv.writer(self.csv)
        if new:
            self.writer.writerow([
                "attempt", "header_stamp_ns", "wall_time_ns", "resolution",
                "detected", "reason", "laplacian_var", "mean_brightness",
                "overexposed_frac", "board_cx", "board_cy", "board_w",
                "board_h", "min_corner_dist_px", "accepted", "saved_path"])
            self.csv.flush()

        qos = QoSProfile(depth=5, reliability=(ReliabilityPolicy.RELIABLE if args.qos == "reliable"
                                                   else ReliabilityPolicy.BEST_EFFORT),
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            Image, args.topic, self.on_image, qos)
        self.get_logger().info(
            f"subscribed {args.topic} ({args.qos}); pattern {PATTERN[0]}x{PATTERN[1]}, "
            f"save dir {images_dir}")

    def on_image(self, msg):
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
        if channels is None or msg.step < msg.width * channels or len(msg.data) != msg.height * msg.step:
            self.state.reason = f"unsupported encoding or invalid layout: {msg.encoding}"
            self.state.ok = False
            self.state.detected = False
            self.state.corners = None
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels).copy()
        if channels == 1:
            img = cv2.cvtColor(img[:, :, 0], cv2.COLOR_GRAY2BGR)
        elif msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.analyze(img, msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)

    def analyze(self, img, stamp_ns):
        st = self.state
        st.img, st.stamp_ns = img, stamp_ns
        st.received_at = time.monotonic()
        st.ok, st.detected, st.corners = False, False, None
        size = (img.shape[1], img.shape[0])
        if self.resolution is None:
            self.resolution = size
            self.get_logger().info(f"Actual image resolution: {size[0]}x{size[1]}")
        elif size != self.resolution:
            st.reason = "resolution changed; start a new session"
            return
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        lap = cv2.Laplacian(gray, cv2.CV_64F).var()
        bright = float(gray.mean())
        over = float((gray >= 250).mean())
        st.metrics = dict(lap=lap, bright=bright, over=over)

        ok, corners = cv2.findChessboardCornersSB(gray, PATTERN, None)
        if not ok:
            st.ok, st.corners = False, None
            st.reason = "corners not found (full 11x8 pattern required)"
            return

        st.detected = True
        c = corners.reshape(-1, 2)
        st.corners = c
        x0, y0 = c.min(axis=0)
        x1, y1 = c.max(axis=0)
        st.metrics.update(board_cx=(x0 + x1) / 2, board_cy=(y0 + y1) / 2,
                          board_w=x1 - x0, board_h=y1 - y0)

        if lap < BLUR_MIN:
            st.ok, st.reason = False, f"too blurry (laplacian var {lap:.0f} < {BLUR_MIN})"
            return
        if not (BRIGHT_MIN <= bright <= BRIGHT_MAX):
            st.ok, st.reason = False, f"brightness {bright:.0f} outside [{BRIGHT_MIN:.0f},{BRIGHT_MAX:.0f}]"
            return
        if over > OVEREXP_FRAC_MAX:
            st.ok, st.reason = False, f"overexposed fraction {over:.3%} > {OVEREXP_FRAC_MAX:.3%}"
            return
        h, w = gray.shape
        if x0 < BORDER_MARGIN_PX or y0 < BORDER_MARGIN_PX or \
           x1 > w - 1 - BORDER_MARGIN_PX or y1 > h - 1 - BORDER_MARGIN_PX:
            st.ok, st.reason = False, "board touches image border"
            return

        min_dist = None
        for i, prev in enumerate(self.saved):
            d = min(float(np.mean(np.linalg.norm(c - ordering, axis=1)))
                    for ordering in (prev, prev[::-1]))
            if min_dist is None or d < min_dist:
                min_dist = d
                nearest = i + 1
        st.metrics["min_corner_dist"] = min_dist
        if min_dist is not None and min_dist < SIMILARITY_PX:
            st.ok, st.reason = False, (
                f"too similar to sample #{nearest} (mean corner dist "
                f"{min_dist:.1f}px < {SIMILARITY_PX}px) — move the board")
            return

        st.ok, st.corners = True, c
        st.reason = "ok"

    def try_save(self):
        st = self.state
        if st.img is not None and time.monotonic() - st.received_at > 2.0:
            st.ok, st.reason = False, "stale image (>2s); check publisher/trigger"
        self.n_attempts += 1
        wall = time.time_ns()
        m = st.metrics
        row = [self.n_attempts, st.stamp_ns, wall,
               f"{st.img.shape[1]}x{st.img.shape[0]}" if st.img is not None else "",
               st.detected, st.reason,
               f"{m.get('lap', 0):.1f}", f"{m.get('bright', 0):.1f}",
               f"{m.get('over', 0):.5f}",
               f"{m.get('board_cx', 0):.1f}", f"{m.get('board_cy', 0):.1f}",
               f"{m.get('board_w', 0):.1f}", f"{m.get('board_h', 0):.1f}",
               f"{m.get('min_corner_dist', 0) or 0:.1f}", False, ""]
        if not st.ok:
            self.writer.writerow(row)
            self.csv.flush()
            return f"SAVE REJECTED: {st.reason}"
        self.n_saved += 1
        name = f"image_{self.n_saved:03d}.png"
        path = os.path.join(self.images_dir, name)
        if not cv2.imwrite(path, st.img):
            self.n_saved -= 1
            row[14], row[15] = False, ""
            row[5] = "imwrite failed"
            self.writer.writerow(row)
            self.csv.flush()
            return "SAVE FAILED (imwrite)"
        self.saved.append(st.corners.copy())
        st.ok, st.reason = False, "already saved; move board and wait for a new frame"
        row[14], row[15] = True, path
        self.writer.writerow(row)
        self.csv.flush()
        return (f"SAVED {name}  (#{self.n_saved}; lap={m['lap']:.0f} "
                f"bright={m['bright']:.0f} center=({m['board_cx']:.0f},{m['board_cy']:.0f}))")

    def draw(self):
        st = self.state
        if time.monotonic() - st.received_at > 2.0:
            st.ok, st.reason = False, "stale image (>2s); check publisher/trigger"
        vis = st.img.copy()
        if st.detected and st.corners is not None:
            for x, y in st.corners:
                cv2.circle(vis, (round(float(x)), round(float(y))), 3, (0, 255, 0), 1)
        if st.ok:
            cv2.putText(vis, "READY - SPACE to save", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(vis, st.reason[:70], (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        m = st.metrics
        cv2.putText(vis, f"lap={m.get('lap', 0):.0f} bright={m.get('bright', 0):.0f} "
                         f"saved={self.n_saved}", (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
        return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/front/image_sync")
    ap.add_argument("--outdir", default=str(Path(__file__).resolve().parent / ("session_" + datetime.now().strftime("%Y%m%d_%H%M%S"))))
    ap.add_argument("--mode", choices=["gui", "headless"], default=None)
    ap.add_argument("--qos", choices=["reliable", "best_effort"], default="reliable")
    args = ap.parse_args()

    rclpy.init()
    node = CaptureNode(args)

    try:
        mode = args.mode or ("gui" if os.environ.get("DISPLAY") else "headless")

        if mode == "gui":
            cv2.namedWindow("capture", cv2.WINDOW_NORMAL)
            print("GUI: SPACE=save  q/ESC=quit")
            while rclcpp_ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.state.img is None:
                    vis = np.zeros((240, 800, 3), dtype=np.uint8)
                    cv2.putText(vis, "Waiting for image; check topic / QoS / camera", (10, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                    cv2.imshow("capture", vis)
                else:
                    cv2.imshow("capture", node.draw())
                k = cv2.waitKey(30) & 0xFF
                if k in (ord('q'), 27):
                    break
                if k == ord(' '):
                    print(node.try_save())
            cv2.destroyAllWindows()
        else:
            print("headless: SPACE+Enter or s+Enter=save, q+Enter=quit (status every 2 s)")
            import select
            next_status = 0.0
            while rclcpp_ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                now = time.time()
                if now >= next_status:
                    st = node.state
                    m = st.metrics
                    print(f"[{now:.0f}] ok={st.ok} saved={node.n_saved} "
                          f"lap={m.get('lap', 0):.0f} bright={m.get('bright', 0):.0f} "
                          f"| {st.reason[:60]}")
                    next_status = now + 2.0
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if r:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    line = line.rstrip("\r\n")
                    if line == "":
                        continue
                    if line[0].lower() == "q":
                        break
                    if line[0] == " " or line[0].lower() == "s":
                        print(node.try_save())

    except KeyboardInterrupt:
        pass
    finally:
        print(f"done: {node.n_saved} images, metadata: {node.csv_path}")
        node.csv.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def rclcpp_ok():
    return rclpy.ok()


if __name__ == "__main__":
    main()
