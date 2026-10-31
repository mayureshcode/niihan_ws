#!/usr/bin/env bash
# Read-only Site Robotics / ROS 2 Jazzy simulator preflight. No motor commands.
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-188}"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not in PATH. Source /opt/ros/jazzy/setup.bash and your workspace install/setup.bash first." >&2
  exit 2
fi
echo "Site Robotics preflight | ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "This tool does NOT enable or drive the rover."
check_topic() {
  local topic="$1"
  echo
  echo "=== $topic ==="
  timeout 6 ros2 topic info "$topic" || echo "WARNING: $topic unavailable / ROS discovery timeout"
}
sample() {
  local topic="$1"
  echo
  echo "=== Sample: $topic ==="
  timeout 7 ros2 topic echo "$topic" --once || echo "WARNING: no sample from $topic in 7 seconds"
}
check_topic /niihan/legacy/lidar/points
check_topic /niihan/sensors/lidar/points
check_topic /scan
check_topic /odom
check_topic /niihan/sensors/panoramic/front/image_raw
sample /niihan/sensor_adapter/status
sample /niihan/health
sample /niihan/motion_gateway/status
echo
echo "Interpretation: legacy LiDAR must have a publisher, adapter status"
echo "must show published_scans increasing, and health ready=true before motion."
echo "If cloud arrives but adapter reports TF failures, inspect timestamped TF."
echo "Low Gazebo real-time factor can make otherwise valid sensors stale."
