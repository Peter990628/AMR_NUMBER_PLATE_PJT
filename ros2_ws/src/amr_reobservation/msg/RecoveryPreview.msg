# Visualization/review output only. No candidate is a certified navigation goal.
# header: original scan timestamp, fixed frame. request_header: context timestamp.
std_msgs/Header header
std_msgs/Header request_header
string request_id
string target_id
# Fresh context + scan + exact-time transforms + supported observed aim(s).
# Does NOT mean reachable, collision-free, or successful plate observation.
bool valid
string status
geometry_msgs/Point vehicle_hint
geometry_msgs/Pose approach_pose
geometry_msgs/Pose current_pose
# Observed connected contour associated with the hint, not a semantic car mask.
# A1/A2/... are frame-local labels; no persistent target IDs or rear/front labels.
string contour_status
uint32 contour_cluster_count
geometry_msgs/Point[] contour_points
uint32[] contour_beam_indices
# Parallel aim arrays. Points use camera height to represent XY bearings only.
string[] aim_labels
geometry_msgs/Point[] aim_points
geometry_msgs/Point[] aim_endpoint_a
geometry_msgs/Point[] aim_endpoint_b
uint32[] aim_support_counts
float64[] aim_observed_lengths_m
float64[] aim_max_deviations_m
# All 6 x aim_count pairs, aim-major. Parallel arrays; empty when valid=false.
# Base positions are hint-relative and shared between different aiming points.
# No selected navigation goal is implied. All poses are BASE poses.
string[] labels
uint32[] candidate_aim_indices
geometry_msgs/Pose[] base_poses
geometry_msgs/Point[] camera_positions
# Observed XY aiming point projected to each camera's height for bearing.
# This is NOT an estimated plate position/height.
geometry_msgs/Point[] look_at
# Passes limited geometric rejection tests; free space/path remain unchecked.
bool[] eligible
string[] reasons
float64[] min_observed_distance_m
# Which aim's six candidates RViz currently shows; -1 if unavailable/invalid.
# Display choice only; not a score, rank, or confirmed motion goal.
int32 marker_aim_index
# Supplemental live observation status, including stale/missing input.
string plate_status
