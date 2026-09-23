#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam_env.sh"
exec "$SLAM_PROJECT_ROOT/ws_slam/reader_build/check_lightning_bag" "${1:?Supply complete SQLite3 bag directory}"
