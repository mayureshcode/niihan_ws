#!/usr/bin/env bash
# One build + one launch: repaired GLIM, upright dashboard, spawn-fixed seaport,
# and ROS 2 Jazzy Nav2. No changes to the original workspace or OS packages.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project="$(dirname "$root")"
orig="$project/niihan_ws"
glim="$project/niihan_glim_repair"
assets="$project/gazebo_assets"
world="$assets/site_robotics_seaport_spawnfix.sdf"
spawn_json="$assets/site_robotics_seaport_spawnfix.json"
[[ "$EUID" -ne 0 ]] || { echo "Run as your regular user (not sudo)." >&2; exit 2; }

# Source ROS in a clean shell; don't inherit one of the old conflicting overlays.
if [[ "${NIIHAN_UNIFIED_FRESH:-0}" != 1 ]]; then
  exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION \
    NIIHAN_UNIFIED_FRESH=1 bash --noprofile --norc "$root/scripts/start_seaport.sh" "$@"
fi

for path in /opt/ros/jazzy/setup.bash \
    "$orig/install/local_setup.bash" \
    "$glim/install/local_setup.bash" \
    "$glim/scripts/environment.sh" "$world" "$spawn_json"; do
  [[ -f "$path" ]] || { echo "Required file missing: $path" >&2; exit 2; }
done
command -v colcon >/dev/null || { echo "Missing colcon build tool." >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$orig/install/local_setup.bash"
source "$glim/install/local_setup.bash"
source "$glim/scripts/environment.sh"
export ROS_DOMAIN_ID=188
export GZ_SIM_RESOURCE_PATH="$assets${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# Refuse an accidental second dashboard; a stale server could look like new output.
if python3 - <<'PY'
import socket
s=socket.socket()
s.settimeout(0.2)
try:
    raise SystemExit(0 if s.connect_ex(('127.0.0.1', 8080)) == 0 else 1)
finally:
    s.close()
PY
then
  echo "Port 8080 is already occupied. Stop the OLD launch from its terminal first." >&2
  exit 2
fi

glim_prefix="$(ros2 pkg prefix glim_ros)"
[[ "$glim_prefix" == "$glim/install/glim_ros" ]] || {
  echo "Wrong GLIM overlay: $glim_prefix; expected $glim/install/glim_ros" >&2
  exit 2
}
binary="$glim/install/glim_ros/lib/glim_ros/glim_rosnode"
[[ -x "$binary" ]] || { echo "Repaired GLIM executable is missing." >&2; exit 2; }
report="$(ldd -r "$binary" 2>&1)" || {
  echo "$report" >&2
  echo "GLIM linker still fails; aborting before Gazebo launch." >&2;exit 2;
}
if grep -Eq 'not found|undefined symbol' <<<"$report"; then
  echo "$report" >&2
  echo "GLIM has unresolved symbols; aborting." >&2;exit 2
fi

mkdir -p "$root/.run/logs"
log="$root/.run/logs/seaport-$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "$log") 2>&1
echo "========== SITE ROBOTICS UNIFIED START =========="
echo "Log: $log"
echo "Repaired GLIM: $glim_prefix"
echo "Building camera + spawn + Nav2 ROS integrations (NOT recompiling GLIM/GTSAM)"
cd "$root"
colcon build --symlink-install \
  --base-paths src/niihan_description src/niihan_slam src/niihan_dashboard \
  --packages-select niihan_description niihan_slam niihan_dashboard \
  --parallel-workers 1
source "$root/install/local_setup.bash"

for pkg in niihan_description niihan_slam niihan_dashboard; do
  actual="$(ros2 pkg prefix "$pkg")"
  expected="$root/install/$pkg"
  [[ "$actual" == "$expected" ]] || {
    echo "Wrong $pkg overlay: $actual (expected $expected)" >&2
    exit 2
  }
  echo "$pkg -> $actual"
done
[[ "$(ros2 pkg prefix glim_ros)" == "$glim_prefix" ]] || {
  echo "GLIM overlay changed unexpectedly." >&2; exit 2;
}

if grep -q 'cv2.ROTATE_180' "$root/src/niihan_dashboard/niihan_dashboard/ros_bridge.py"; then
  echo "Camera rotation regression detected. Aborting." >&2; exit 2
fi
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider \
  "$root/src/niihan_slam/test/test_nav2_launch_profile.py" \
  "$root/src/niihan_slam/test/test_unified_seaport_launch.py"
python3 -m py_compile \
  "$root/src/niihan_slam/launch/slam_simulation.launch.py" \
  "$root/src/niihan_description/launch/niihan_navigation.launch.py"
coords="$(python3 - "$spawn_json" <<'PY'
import json, math, sys
from pathlib import Path
xyz=json.loads(Path(sys.argv[1]).read_text())['spawn_xyz']
assert len(xyz)==3 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in xyz)
print(*xyz)
PY
)" || { echo "Invalid spawn_xyz JSON" >&2; exit 2; }
read -r spawn_x spawn_y spawn_z <<<"$coords"
echo "World: $world"
echo "Validated robot spawn: $spawn_x, $spawn_y, $spawn_z"
echo "Camera: no backend 180-degree rotation"
echo "Nav2 profile: stock Jazzy; startup deferred 15 seconds for GLIM bootstrap."
echo "Dashboard: http://localhost:8080"
echo "================================================"

# Report readiness automatically; this does not activate nodes or command motion.
(
  sleep 45
  echo "========== NAV2 / SLAM AUTOMATIC CHECK =========="
  if ! timeout 25 python3 "$root/scripts/nav2_lifecycle_report.py"; then
    echo "NAV2 NOT READY. Review FIRST lifecycle error above / in $log."
    echo "Motion remains controlled by the existing health/e-stop gateway."
  fi
  echo "=================================================="
) &
watcher=$!
cleanup() { kill "$watcher" 2>/dev/null || true; }
trap cleanup EXIT

ros2 launch niihan_slam slam_simulation.launch.py \
  world:="$world" mode:=mapping headless:=false rviz:=false \
  spawn_x:="$spawn_x" spawn_y:="$spawn_y" spawn_z:="$spawn_z" \
  nav2_bt_profile:=stock nav2_start_delay:=15.0 \
  environment_id:=sea-port-mapping-v1 seed:=42
