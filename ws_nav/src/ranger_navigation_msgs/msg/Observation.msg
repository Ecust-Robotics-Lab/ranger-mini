# Sampling stamp and all geometry are in the odom frame, from the same scan.
std_msgs/Header header
string session_id
geometry_msgs/Point32[] obstacles
float32[] visible_ranges
# Pose of the local observation grid in odom (yaw zero, snapped to 0.05m).
# Actual LiDAR ray origin is loaded from the shared installation geometry.
geometry_msgs/Pose2D sensor_pose
# 80x80 cells in sensor_pose's grid frame, origin [-2,-2], resolution 0.05.
# 0=observed; -1=unknown. Raw observation evidence expires after <=2 seconds.
int8[6400] observed_cells
float32 voxel_size
bool valid
string reason
