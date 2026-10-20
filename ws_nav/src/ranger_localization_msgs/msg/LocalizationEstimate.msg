# Atomic map-frame body pose and backend health. No controller velocity.
# header.stamp is the original pose sampling time, NEVER the heartbeat time.
# Repeated heartbeats retain the same stamp; consumers enforce freshness.
std_msgs/Header header
string base_frame
geometry_msgs/Pose pose
uint8 INITIALIZING=0
uint8 VALID=1
uint8 DEGRADED=2
uint8 LOST=3
uint8 ERROR=4
uint8 state
string reason
