#!/usr/bin/env python3
"""Read-only inventory; never starts ROS nodes, drivers or changes calibration."""
import argparse
import hashlib
import json
import platform
import sqlite3
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def command(args, cwd=ROOT):
    p = subprocess.run(args, cwd=cwd, text=True, capture_output=True)
    return {"command": args, "exit_code": p.returncode,
            "stdout": p.stdout, "stderr": p.stderr}


def inventory():
    repos = [ROOT, *sorted((ROOT / "vendor/fast_livo2_review").glob("*/.git"))]
    repos = [p.parent if p.name == ".git" else p for p in repos]
    repos += [p.parent for w in ("ws_base", "ws_camera", "ws_livox", "ws_slam")
              for p in (ROOT / w / "src").glob("*/.git")]
    repos += [ROOT / "calibration/lidar_camera_extrinsics"]
    versions = []
    for p in repos:
        if not (p / ".git").exists():
            continue
        versions.append({"path": str(p), **{name: command(args, p) for name, args in {
            "commit": ["git", "rev-parse", "HEAD"],
            "branch": ["git", "symbolic-ref", "--short", "-q", "HEAD"],
            "status": ["git", "status", "--porcelain=v1"],
            "remote": ["git", "remote", "-v"],
            "diff": ["git", "diff", "--stat"],
        }.items()}, "licenses": [{"path": str(f.relative_to(p)),
                                    "sha256": hashlib.sha256(f.read_bytes()).hexdigest()}
                                   for f in sorted(p.rglob("*LICENSE*")) if f.is_file()]})
    assets = []
    for p in [ROOT / "AGENTS.md", ROOT / "config/runtime.yaml",
              ROOT / "docs/03_mid360_validation.md",
              *sorted((ROOT / "calibration/camera_intrinsics/results").glob("*.yaml")),
              ROOT / "calibration/ws_calib/src/hik_camera_trigger_driver/src/hik_camera_node.cpp",
              ROOT / "calibration/ws_calib/src/timeshare_writer/src/timeshare_writer_node.cpp",
              ROOT / "calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407/joint/multi_calib_result.txt"]:
        if p.is_file():
            assets.append({"path": str(p), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()})
    bags = []
    for parent in (ROOT / "bags", ROOT / "calibration"):
        for meta in sorted(parent.rglob("metadata.yaml")):
            counts = []
            for db in sorted(meta.parent.glob("*.db3")):
                with sqlite3.connect(db.as_uri() + "?mode=ro", uri=True) as conn:
                    counts.append({"path": str(db), "size": db.stat().st_size,
                                   "topics": conn.execute(
                                       "SELECT t.name,t.type,count(m.id),min(m.timestamp),max(m.timestamp) "
                                       "FROM topics t LEFT JOIN messages m ON t.id=m.topic_id GROUP BY t.id"
                                   ).fetchall()})
            bags.append({"metadata": str(meta), "sha256": hashlib.sha256(meta.read_bytes()).hexdigest(),
                         "databases": counts})
    return {"generated_at": datetime.now(timezone.utc).isoformat(),
            "machine": platform.uname()._asdict(), "versions": versions,
            "assets": assets, "bags": bags,
            "processes": command(["ps", "-eo", "pid,ppid,comm"]),
            "os": Path("/etc/os-release").read_text()}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = inventory()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(args.out)
