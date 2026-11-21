#!/usr/bin/env bash
# One-terminal entry point for the Isaac Sim recovery experiment.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -r "$project_dir/ros2_ws/install/setup.bash" ]]; then
  echo "ROS 작업공간을 먼저 빌드해 주세요: $project_dir/ros2_ws" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$project_dir/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

exec /usr/bin/python3 "$project_dir/tools/run_plate_recovery.py" \
  --mock-params "$project_dir/ros2_ws/src/amr_reobservation/config/camera_fail_test_hint.yaml" \
  --record-sensors "$@"
