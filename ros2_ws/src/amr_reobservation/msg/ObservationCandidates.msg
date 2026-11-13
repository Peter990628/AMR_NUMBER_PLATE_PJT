# Geometric sensor viewpoints, NOT base_link navigation goals or rear-end detections.
# Coordinates stay in the source LaserScan frame. Check valid AND freshness.
std_msgs/Header header
bool valid
string status
float64 extension_m
float64 standoff_m
# Empty when invalid. A then B when valid; labels are not tracked object identities.
geometry_msgs/Pose[] viewpoints
# Observed line endpoints to face, not known plate positions or bumper extremities.
geometry_msgs/Point[] look_at
