# C1 offline prior-map localizer

Read-only B graph archive, operator initial pose, 6DoF geometric confirmation and
continuous map matching. Default C1 requires a new operator initial pose after
LOST. Optional C2 global initialization/recovery is described below. Neither
entry starts devices or a base, broadcasts public TF, or feeds back to the frontend.

Build from the project root with `bash scripts/build_fast_livo2_localizer.sh
--execute`; omit `--execute` for the default dry-run. Dependencies reuse the
locked project-local B prefix. The production runtime remains Lightning.

`bash scripts/run_fast_livo2_localizer.sh` describes the managed entry. Explicit
`--execute --archive ABS_DIR --compatibility ABS_JSON` starts the private offline
service in an isolated localhost domain with simulated time. Only explicit
`--offline-bridge` adds the C1 adapter and existing public bridge; the caller must
supply synthetic wheel after the occupancy check.

Services: `/fast_livo/localizer/load_map` and `/fast_livo/localizer/initialize`.
Atomic state/confirmed pose: `/fast_livo/localizer/estimate`. Acceptance of a
service request is not successful localization. Initial stamp zero explicitly
binds to the next paired frontend sample. Other timestamps require same-session
bounded history with no extrapolation.

See [interface and operations](../../../docs/fast_livo2_localizer_interfaces.md)
and [C1 report](../../../docs/fast_livo2_stage_c1_report.md). The same-capture
check is not independent-session or absolute-accuracy validation.


Optional offline C2: build with `bash scripts/build_fast_livo2_relocalizer.sh --execute`
(default dry-run) and run `bash scripts/run_fast_livo2_relocalizer.sh` to inspect the
managed entry. The explicit C2 overlay enables full 6DoF candidate geometry,
disjoint moving query support and handoff to the same C1 tracker. The C1 build
script explicitly disables the optional KISS dependency again. See
[relocalization contract](../../../docs/fast_livo2_relocalizer_interfaces.md) and
[C2 report](../../../docs/fast_livo2_stage_c2_report.md). Read dependency notices:
KISS transitively links GPLv3+ PMC; this optional binary is not MIT-only.
