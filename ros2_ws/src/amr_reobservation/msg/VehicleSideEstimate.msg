# A linear surface candidate in the LaserScan frame, NOT a semantic vehicle detection.
# header is the input scan acquisition time. Consumers must check valid AND freshness.
std_msgs/Header header
bool valid
string status
uint32 input_point_count
uint32 cluster_count

# The following fields are zero/default when valid=false.
# All coordinates are on the scan plane (z=0), in header.frame_id.
geometry_msgs/Point center
geometry_msgs/Point endpoint_a
geometry_msgs/Point endpoint_b
# Unit axis with deterministic sign; NOT the vehicle's forward direction.
geometry_msgs/Vector3 tangent
# Perpendicular unit vector oriented toward the LiDAR origin.
geometry_msgs/Vector3 normal_toward_sensor
# Span of the observed inlier projections; NOT the full vehicle length.
float64 observed_length_m
float64 rmse_m
float64 inlier_ratio
uint32 inlier_count
