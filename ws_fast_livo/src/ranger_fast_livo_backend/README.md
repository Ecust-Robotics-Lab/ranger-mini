# Ranger FAST-LIVO2 independent backend

C++17 / ROS 2 Humble. The node consumes only the paired VIO-corrected IMU pose
and single deskewed IMU-frame interval cloud. It never broadcasts navigation TF
or modifies the frontend filter/visual map.

Build and dry-run entrypoints (from the project root):

```bash
bash scripts/build_fast_livo2_backend.sh --execute
bash scripts/run_fast_livo2_backend.sh
python3 scripts/fast_livo2/backend/run_checks.py
```

See `docs/fast_livo2_backend_interfaces.md` for transforms, thresholds, session
and snapshot semantics, and `docs/fast_livo2_stage_b_report.md` for actual test
commands, results and limits. Default production navigation remains Lightning.
C1/C2 map loading and relocalization are not implemented.

Recorded-motion validation is available via
`python3 scripts/fast_livo2/backend/validate_motion.py --native PATH --output NEW_PATH`
(dry-run; `--execute` starts the backend only). It publishes the original paired
messages with a simulated clock and waits for each actual backend acknowledgment.
See `docs/fast_livo2_stage_b_motion_report.md` for the 1362-pair PCL/GICP results,
snapshot integrity checks and their limits. Wall-time pacing is not a real-time benchmark.

Messages and services are private to this package. The optional existing bridge
adapter requires sourcing `ws_fast_livo/backend_install/setup.bash`; generic
Lightning navigation does not import backend messages.

Third-party sources are pinned in `config/fast_livo2/sources.lock.json`.
See LICENSE, THIRD_PARTY_NOTICES.md and the retained texts in `third_party/`.
