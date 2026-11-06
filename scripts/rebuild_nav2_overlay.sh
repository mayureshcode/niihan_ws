#!/usr/bin/env bash
# Rebuild only NIIHAN's Nav2 launch/SLAM integration in this separate worktree.
# The already repaired GLIM and GTSAM native binaries are reused unmodified.
set -eo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
parent="$(dirname "$root")"
original="$parent/niihan_ws"
waypoint="$parent/niihan_waypoint_review"
glim="$parent/niihan_glim_repair"

[[ $EUID -ne 0 ]] || { echo "Run as your normal user, not sudo." >&2; exit 2; }
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS Jazzy setup missing." >&2; exit 2; }
[[ -f "$original/install/local_setup.bash" ]] || { echo "Original NIIHAN overlay missing." >&2; exit 2; }
[[ -f "$glim/install/local_setup.bash" ]] || { echo "Repaired GLIM overlay missing: $glim" >&2; exit 2; }
[[ -f "$glim/scripts/environment.sh" ]] || { echo "Pinned GLIM GTSAM environment missing." >&2; exit 2; }

# A fresh build shell avoids finding stale packages from unrelated sourced overlays.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION
source /opt/ros/jazzy/setup.bash
source "$original/install/local_setup.bash"
if [[ -f "$waypoint/install/local_setup.bash" ]]; then
  source "$waypoint/install/local_setup.bash"
fi
source "$glim/install/local_setup.bash"
source "$glim/scripts/environment.sh"

glim_prefix="$(ros2 pkg prefix glim_ros)"
[[ "$glim_prefix" == "$glim/install/glim_ros" ]] || {
  echo "Wrong GLIM prefix: $glim_prefix (expected $glim/install/glim_ros)" >&2
  exit 1
}

cd "$root"
colcon build --symlink-install \
  --base-paths src/niihan_description src/niihan_slam \
  --packages-select niihan_description niihan_slam

source "$root/install/local_setup.bash"
for package in niihan_description niihan_slam; do
  actual="$(ros2 pkg prefix "$package")"
  [[ "$actual" == "$root/install/$package" ]] || {
    echo "Wrong $package prefix: $actual" >&2
    exit 1
  }
done

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider \
  "$root/src/niihan_slam/test/test_nav2_launch_profile.py"

echo "PASS: Nav2 integration rebuilt; launch profiles and source tests pass."
echo "GLIM was reused from $glim; runtime Nav2 readiness is NOT yet verified."
echo "Follow docs/NAV2_JAZZY_RECOVERY.md to launch your spawn-fixed seaport."
