# One diagnostic per received scan, including empty/invalid scans.
# Exact original scan header; join by acquisition stamp AND frame (and clock epoch).
# No message is synthesized when scan input stops.
std_msgs/Header header
bool valid
# Same status/valid/counts as VehicleSideEstimate for this input scan.
string status
uint32 input_point_count
uint32 cluster_count
# -1 when no fit is selected.
int32 selected_cluster_index
# Steady wall time for preprocessing + estimation, excluding ROS publication,
# marker generation and input transport latency.
float64 processing_time_ms
ClusterFitDiagnostics[] clusters
