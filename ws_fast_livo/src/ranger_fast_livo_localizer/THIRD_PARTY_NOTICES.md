# C1 dependency and reference notices

This package is independently implemented Ranger code under MIT. It reuses the
project's `ranger_fast_livo_backend` input, geometry, registration and archive
helpers. Exact GTSAM/small_gicp/SimplePGO sources and retained license texts are
in that package's THIRD_PARTY_NOTICES.md and third_party directory. The existing
Ubuntu PCL/Eigen/OpenSSL/nlohmann-json and ROS 2 Humble dependencies are reused.
No new third-party source, global library install, Scan Context, STD, iBTC,
KISS-Matcher, HoloAgent or OpenFleX implementation is introduced by C1.
SimplePGO's two-resolution registration pattern informed the architecture;
its ROS localizer, state management and TF code are not copied. Frontend
licensing remains independent and its distribution questions are unchanged.


## Optional C2 build (RANGER_ENABLE_KISS=ON)

C1's default build disables this option. The independently written C2 retrieval,
index and state code is MIT; the optional linked binary has additional obligations.
Pinned repositories and origins are in config/fast_livo2/relocalization_sources.lock.json.

- KISS-Matcher 8b58b118e00c7a5a22448741f8d956d638a44ae2: root MIT, local C++ core only;
  no ROS SAM framework, Python bindings or CUDA. Includes the retained per-file
  Kenji Koide point helpers and Tessil robin_map MIT notices in installed headers.
- ROBIN 5623f02f62c63711052bbb8d6096a410ddca0ee4: BSD-2-Clause (LICENSE.md also
  retains an unused Catch2 license). No tests, docs or MATLAB bindings built.
- PMC 4bbd40ababd8e925c4e1845c509173afe766b443: **GPL-3.0-or-later**. It is
  statically linked through ROBIN; root KISS MIT does not make this dependency
  MIT. A redistributed linked binary must comply with the applicable GPL terms,
  including corresponding source obligations. This local build is not a complete
  distribution-compliance determination.
- xenium 1c449ae953ce2a440b0d16c5ed1181d2754860ab: MIT. The generated ROBIN
  source projection replaces its upstream moving-main download with this locked
  local tree. Original vendor repositories remain pristine.
- Existing system Eigen/TBB/FLANN/LZ4/OpenMP are reused, not upgraded or replaced.

License texts are retained in third_party/. Source file notices remain in the
locked vendor trees and installed upstream headers. C2 dependencies install only
under ws_fast_livo/relocalization_deps. The build projection generator and source
lock are part of this repository; no unpinned downloads occur during the build.

SC-A-LOAM 321cb4b07b7815c114237ce5d87014d2e46b972c has package BSD metadata but
no observed root or core-file authorization text. Its Scancontext implementation
is not copied or linked. The polar-height selection experiment is independently
implemented mathematics, not the upstream detector. STD/iBTC remain unintegrated.
