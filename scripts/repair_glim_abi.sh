#!/usr/bin/env bash
# Rebuild a matching CPU GLIM/GTSAM stack in THIS worktree only.
# No modification of /usr/local, system packages, or another NIIHAN workspace.
set -Eeuo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ $EUID -ne 0 ]] || { echo "Run as your normal user, not sudo." >&2; exit 2; }
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS 2 Jazzy is required." >&2; exit 2; }
for tool in git cmake colcon c++; do
  command -v "$tool" >/dev/null || { echo "Missing build tool: $tool" >&2; exit 2; }
done
# Do not accidentally compile against a previously sourced, incompatible overlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION
source /opt/ros/jazzy/setup.bash
source "$root/scripts/platform.sh"
niihan_select_platform
[[ "$NIIHAN_ROS_DISTRO" == jazzy ]] || {
  echo "This recovery targets Ubuntu 24.04 / ROS Jazzy only." >&2; exit 2;
}
printf 'Rebuilding pinned GTSAM, gtsam_points, GLIM and glim_ros in:\n%s\n' "$root"
echo "Existing niihan_ws and Gazebo processes will not be modified."
git -C "$root" submodule update --init src/glim src/glim_ros2

# These pinned revisions are specified by the repo's existing dependency builder.
# Never link an old GLIM binary to a mismatched apt GTSAM by creating symlinks.
bash "$root/scripts/build_dependencies.sh"
deps="$root/.deps/jazzy/install"
[[ -d "$deps/lib" ]] || { echo "Pinned GTSAM dependency prefix missing: $deps" >&2; exit 1; }
source "$root/scripts/environment.sh"
export CMAKE_PREFIX_PATH="$deps${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$deps/lib:$deps/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL="${NIIHAN_BUILD_JOBS:-2}"
mkdir -p "$root/log"
colcon --log-base "$root/log/glim-abi-repair" build \
  --base-paths "$root/src/glim" "$root/src/glim_ros2" \
  --build-base "$root/build" --install-base "$root/install" \
  --parallel-workers 1 --packages-select glim glim_ros \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF \
    -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTING=OFF

exe="$root/install/glim_ros/lib/glim_ros/glim_rosnode"
libglim="$root/install/glim/lib/libglim.so"
[[ -x "$exe" && -f "$libglim" ]] || {
  echo "GLIM build produced no expected executable/library." >&2; exit 1;
}
# Require the new GLIM before any pre-existing source install.
export LD_LIBRARY_PATH="$root/install/glim/lib:$root/install/glim_ros/lib:$deps/lib:$deps/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
report="$(ldd -r "$exe" 2>&1)" || {
  printf '%s\n' "$report" >&2
  echo "ELF relocation check failed. Do not launch." >&2; exit 1;
}
if grep -Eq 'not found|undefined symbol' <<<"$report"; then
  printf '%s\n' "$report" >&2
  echo "GLIM still has unresolved dependencies/symbols. Do not launch." >&2; exit 1;
fi
for name in gtsam gtsam_points; do
  actual="$(awk -v name="lib${name}.so" 'index($1, name) == 1 {print $3; exit}' <<<"$report")"
  if [[ "$actual" != "$deps/lib/"* && "$actual" != "$deps/lib64/"* ]]; then
    echo "Unexpected $name library: ${actual:-not listed}; expected pinned prefix $deps" >&2
    echo "Do not use the old system GTSAM with this GLIM build." >&2
    exit 1
  fi
done
echo "PASS: rebuilt GLIM resolves pinned GTSAM/gtsam_points with no unresolved relocations."
echo "To use it, source your original niihan_ws overlay, THEN this worktree's"
echo "install/local_setup.bash and scripts/environment.sh in the launch terminal."
echo "This verifies dynamic linking only: live LiDAR, TF, Nav2 and motion remain untested."
