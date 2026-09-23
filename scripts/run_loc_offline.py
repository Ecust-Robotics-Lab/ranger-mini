#!/usr/bin/env python3
"""Stage 5 offline localization validation runner (dry-run by default).

Runs Lightning run_loc_offline against an accepted map experiment and a bag.
Writes evidence under docs/05_evidence/loc-offline-<id>/ (manifest, config
snapshot, run.log, result.json). Does not start any driver, does not touch the
map directory (config must keep save_dyn_when_quit=false; the runner verifies).
"""
import argparse
import hashlib
import json
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
EVIDENCE = ROOT / "docs/05_evidence"
EXE = ROOT / "ws_slam/build/lightning/bin/run_loc_offline"


def digest(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=ROOT / "config/slam/ranger_mid360s_loc.yaml")
    ap.add_argument("--map-exp", type=Path, required=True,
                    help="Experiment directory with data/new_map and acceptance.json")
    ap.add_argument("--bag", type=Path, required=True, help="Bag raw directory to replay offline")
    ap.add_argument("--motion-note", required=True, help="Data provenance and purpose of this validation")
    ap.add_argument("--execute", action="store_true")
    args = ap.parse_args()

    cfg = args.config.resolve()
    map_exp = args.map_exp.resolve()
    bag = args.bag.resolve()
    map_dir = map_exp / "data/new_map"

    problems = []
    if not EXE.is_file():
        problems.append("run_loc_offline binary missing; build Lightning first")
    if not cfg.is_file():
        problems.append(f"config not found: {cfg}")
    if not (map_dir / "index.txt").is_file():
        problems.append(f"map index.txt not found under {map_dir}")
    acc = map_exp / "acceptance.json"
    if acc.is_file():
        review = json.loads(acc.read_text())
        if review.get("three_dimensional") not in ("PASS", "PASS_BY_REFERENCE"):
            problems.append(f"map experiment not accepted for 3D: {review.get('three_dimensional')}")
    else:
        problems.append(f"acceptance.json missing in {map_exp}")
    if not (bag / "metadata.yaml").is_file():
        problems.append(f"not a bag raw directory: {bag}")
    text = cfg.read_text()
    if "save_dyn_when_quit: false" not in text:
        problems.append("config must set maps.save_dyn_when_quit: false to protect the map directory")

    print(json.dumps({"config": str(cfg), "map_experiment": str(map_exp), "bag": str(bag),
                      "executable": str(EXE), "problems": problems, "execute": args.execute},
                     ensure_ascii=False))
    if problems:
        return 2
    if not args.execute:
        return 0

    EVIDENCE.mkdir(exist_ok=True)
    exp = Path(tempfile.mkdtemp(prefix="loc-offline-", dir=EVIDENCE))
    (exp / "data").mkdir()
    cfg_copy = exp / "config.yaml"
    cfg_copy.write_text(text)
    cmd = [str(EXE), f"--config={cfg_copy}", f"--map_path={map_dir}", f"--input_bag={bag}"]
    manifest = {"created_local": time.strftime("%Y-%m-%dT%H:%M:%S"), "command": cmd, "cwd": str(exp),
                "bag": str(bag), "map_experiment": str(map_exp), "motion_note": args.motion_note,
                "executable_sha256": digest(EXE), "config_sha256": digest(cfg_copy)}
    (exp / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))

    result = {}
    try:
        with (exp / "run.log").open("w") as f:
            p = subprocess.run(cmd, cwd=exp, stdout=f, stderr=subprocess.STDOUT, timeout=3600)
            result["exit"] = p.returncode
    except subprocess.TimeoutExpired:
        result["exit"] = "TIMEOUT"
    for extra in ["data/recover_pose.txt"]:
        q = exp / extra
        if q.is_file():
            result[extra] = q.read_text().strip()
    (exp / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print("EVIDENCE=" + str(exp))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get("exit") == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
