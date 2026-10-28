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
