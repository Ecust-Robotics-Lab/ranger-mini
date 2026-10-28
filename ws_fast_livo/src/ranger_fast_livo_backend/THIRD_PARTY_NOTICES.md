# Algorithm references and dependency notices

The Ranger backend is an independent package. SimplePGO's keyframe / relative
odometry / submap / iSAM2 organization informed its implementation. Its ROS node,
message synchronization, acceptance policy, persistence and TF implementation
are not used. The upstream MIT permission text is retained in `third_party/`.

| Component | Exact source | Use | License |
| --- | --- | --- | --- |
| SimplePGO | liangheming/FASTLIO2_ROS2, 7baa59f7339ba88ce3b646fe35b23f61031c5d3f, pgo/src/pgos | Algorithm reference | MIT, upstream package text retained |
| GTSAM | borglab/gtsam, 4f66a491ffc83cf092d0d818b11dc35135521612 | Project-local linked dependency | BSD and separately licensed bundled components; see retained upstream LICENSE and LICENSE.BSD |
| small_gicp | koide3/small_gicp, fa0cfc983417c044b6036f71dfa448038bb9c820 | Project-local GICP/VGICP dependency | MIT, text retained |
| KISS-Matcher | MIT-SPARK/KISS-Matcher, 8b58b118e00c7a5a22448741f8d956d638a44ae2 | Reviewed coarse/fine registration and candidate separation; no copied solver or SAM | MIT observed in upstream root |

PCL and Eigen use the existing Ubuntu packages; this package does not bundle or
replace them. All original FAST-LIVO2/frontend licenses remain applicable to
those separate components. This notice does not resolve distribution licensing
for the existing frontend combination.

SC-A-LOAM, STD and iBTC are future retrieval references, not dependencies or
copied implementations. In particular STD's README states GPLv2 together with a
personal/academic restriction and commercial-contact wording; package.xml is
TODO. iBTC's README states GPLv2 while package.xml is TODO. Do not infer blanket
reuse permission from repository availability or package metadata.
