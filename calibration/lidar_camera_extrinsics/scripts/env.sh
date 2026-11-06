# Source this file, do not execute it.
_C2_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$_C2_ROOT/ws/install_integrated/fast_calib/share/fast_calib/local_setup.bash"
export ROS_LOG_DIR="$_C2_ROOT/logs/ros"
export MPLCONFIGDIR="$_C2_ROOT/logs/matplotlib"
unset _C2_ROOT
