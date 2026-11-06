#!/usr/bin/env bash
# Read-only ROS 2 / Gazebo navigation diagnostics. Does not alter nodes, topics or safety gates.
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-188}"
printf 'NIIHAN Nav2 diagnostic (ROS_DOMAIN_ID=%s)\n' "$ROS_DOMAIN_ID"
printf 'Run while slam_simulation.launch.py is active.\n\n'
if ! command -v ros2 >/dev/null 2>&1; then
  echo 'ERROR: ros2 is not available. Source /opt/ros/jazzy/setup.bash and workspace/install/local_setup.bash.'
  exit 2
fi
run() { printf '\n### %s\n' "$1"; shift; timeout 12 "$@" 2>&1 || true; }
run 'ROS node presence (GLIM must appear)' bash -c 'ros2 node list | grep -E "glim_ros|gazebo_drive|slam_map_bridge|lio_tf_filter|controller_server|planner_server|bt_navigator|lifecycle_manager" || true'
run 'Wheel odometry endpoints' ros2 topic info /niihan/wheel/odom -v
run 'GLIM odometry endpoints (zero publishers => GLIM absent)' ros2 topic info /glim_ros/odom -v
run 'Navigation odometry endpoints' ros2 topic info /odom -v
run 'Wheel odometry sample' bash -c 'timeout 5 ros2 topic echo /niihan/wheel/odom --once'
run 'GLIM odometry sample' bash -c 'timeout 5 ros2 topic echo /glim_ros/odom --once'
run 'Navigation odometry sample' bash -c 'timeout 5 ros2 topic echo /odom --once'
run 'LiDAR endpoint' ros2 topic info /niihan/sensors/unitree_lidar/points -v
run 'IMU endpoint' ros2 topic info /niihan/imu/data -v
run 'Nav2 controller state' ros2 lifecycle get /controller_server
run 'Nav2 planner state' ros2 lifecycle get /planner_server
run 'Nav2 navigator state' ros2 lifecycle get /bt_navigator
run 'GLIM process' bash -c "pgrep -af '[g]lim_rosnode|[g]lim_ros' || true"
run 'GLIM launch log errors (if accessible)' bash -c 'if [ -d "$HOME/.ros/log/latest" ]; then grep -RiE "glim_ros|process has died|Segmentation fault|bad_alloc|pluginlib|exception" "$HOME/.ros/log/latest" | tail -50; else echo "No latest ROS launch log directory"; fi'
run 'GLIM executable and dynamic linker diagnostics (exit 127)' bash -c '
  exe="$(ros2 pkg prefix glim_ros 2>/dev/null)/lib/glim_ros/glim_rosnode"
  if [ ! -x "$exe" ]; then echo "Missing or non-executable: $exe"; exit 0; fi
  echo "Executable: $exe"
  file "$exe"
  echo "--- Missing shared libraries (ldd) ---"
  ldd "$exe" 2>&1 | grep -E "not found|GLIM|gtsam|cuda|tbb|gomp|boost|open3d|pcl|ceres|opencv" || true
  echo "--- All unresolved dependencies from loader ---"
  ldd "$exe" 2>&1 | grep "not found" || true
  echo "Check complete. Do not copy random .so files or alter LD_LIBRARY_PATH yet."
'
echo
echo 'Interpretation: If /glim_ros/odom has zero publishers, inspect the GLIM process/logs and sensor sources first.'
echo 'If GLIM publishes but /odom is empty, inspect slam_map_bridge and its base-frame TF.'
echo 'If navigation odometry is live but Nav2 stays inactive, inspect lifecycle manager startup errors.'
echo 'No odometry fallback, forced lifecycle transition, or motion interlock override is performed.'
