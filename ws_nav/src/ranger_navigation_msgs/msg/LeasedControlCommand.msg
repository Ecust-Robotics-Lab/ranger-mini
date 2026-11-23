# Original reference sampling stamp, never refreshed by a heartbeat.
std_msgs/Header header
string session_id
uint64 generation
uint64 request_id
# Host CLOCK_MONOTONIC deadline; zero is invalid for autonomous commands.
float64 valid_until_steady
uint8 mode
geometry_msgs/Twist twist
bool valid
string reason
