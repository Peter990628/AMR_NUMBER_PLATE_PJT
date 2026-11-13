# Mission context for geometric preview, NOT a navigation request.
# header uses the ROS clock and a stationary fixed frame (map/odom), not a
# moving sensor/base frame. A clock reset requires a new request.
std_msgs/Header header
string request_id
string target_id
bool enabled
# Approximate vehicle observation point; not guaranteed center or plate position.
# XY only. Vehicle heading is UNKNOWN; approach yaw must never substitute for it.
geometry_msgs/Point vehicle_hint
# Original BASE approach pose, in the same fixed frame. XY+yaw only.
geometry_msgs/Pose approach_pose
string failure_reason
# Lifetime from header.stamp in ROS seconds; identical retries do not renew it.
float64 valid_for_sec
# E.g. upstream mission adapter or explicitly labelled manual/mock input.
string source
