#!/usr/bin/env bash
# Isolated, read-only offline checks. Original installed node is not replaced.
cd /home/lab307/ranger_nav
source /opt/ros/humble/setup.bash
source ws_fast_livo/backend_install/setup.bash
source ws_fast_livo/install_c1_confirmation_20261104/setup.bash
export LD_LIBRARY_PATH=/home/lab307/ranger_nav/ws_fast_livo/backend_deps/lib:/home/lab307/ranger_nav/ws_fast_livo/relocalization_deps/lib:/home/lab307/ranger_nav/ws_fast_livo/apriltag_deps/lib:${LD_LIBRARY_PATH:-}
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
python3 docs/fast_livo2_evidence/tag_two_capture_20261104/validate_optimized_node.py \
 --archive outputs/fast-livo-tag140-two-capture-20261104/new_local_map/snapshot \
 --compatibility docs/fast_livo2_evidence/tag_two_capture_20261104/new_map_compatibility.json \
 --native outputs/fast-livo-tag140-two-capture-20261104/straight/livo_110000000/result/native_frames \
 --query-provenance docs/fast_livo2_evidence/tag_two_capture_20261104/straight_new_map_relation.json \
 --output outputs/fast-livo-tag140-two-capture-20261104/straight/c2_new_map_optimized \
 --domain 203 --initial-index 0 --global-search --allow-recovery --execute \
 > docs/fast_livo2_evidence/tag_two_capture_20261104/straight_c2_new_map_optimized.log 2>&1
straight_code=$?
printf 'straight_exit=%s\n' "$straight_code"
python3 docs/fast_livo2_evidence/tag_two_capture_20261104/validate_optimized_node.py \
 --archive outputs/fast-livo-tag140-two-capture-20261104/new_local_map/snapshot \
 --compatibility docs/fast_livo2_evidence/tag_two_capture_20261104/new_map_compatibility.json \
 --native outputs/fast-livo-tag140-two-capture-20261104/turn/livo_110000000/result/native_frames \
 --query-provenance docs/fast_livo2_evidence/tag_two_capture_20261104/turn_new_map_relation.json \
 --output outputs/fast-livo-tag140-two-capture-20261104/turn/c1_new_map_optimized \
 --domain 204 --initial-index 267 --execute \
 > docs/fast_livo2_evidence/tag_two_capture_20261104/turn_c1_new_map_optimized.log 2>&1
turn_code=$?
printf 'turn_exit=%s\n' "$turn_code"
printf '{"straight_c2_exit":%s,"turn_training_c1_exit":%s}\n' "$straight_code" "$turn_code" > docs/fast_livo2_evidence/tag_two_capture_20261104/new_map_optimized_exits.json
exit $((straight_code || turn_code))
