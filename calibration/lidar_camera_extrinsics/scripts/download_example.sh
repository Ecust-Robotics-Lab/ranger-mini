#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
DEST="$ROOT/datasets/upstream_mid360_11"
mkdir -p "$DEST"
BASE=https://raw.githubusercontent.com/ichangjian/FAST-Calib-ROS2/dev_humble/calib_data/mid360_11
for item in metadata.yaml mid360_11_0.db3 11.png; do
  if [[ -e "$DEST/$item" ]]; then echo "Refuse overwrite: $DEST/$item"; exit 2; fi
  curl --fail --location --max-time 45 "$BASE/$item" -o "$DEST/$item.partial"
  mv "$DEST/$item.partial" "$DEST/$item"
done
# Dataset belongs to ordinary Mid360, only upstream regression; never substitute its K,D for the real camera.
