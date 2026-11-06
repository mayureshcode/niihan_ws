# ROS 2 Jazzy Nav2 lifecycle recovery (Site Robotics seaport)

## What the reported logs prove

After the GLIM ABI repair, the following were observed on ROS_DOMAIN_ID=188:

- `/glim_ros` running, `/glim_ros/odom` with one publisher.
- `/niihan/health`: `ready: true`, `faults: []`.
- `controller_server`, `planner_server`, `behavior_server`: `inactive [2]`.
- `bt_navigator`: `unconfigured [1]`.
- `velocity_smoother`: lifecycle CLI raised an AttributeError before reporting a usable state.
- `/navigate_to_pose` appears in the action list; this **does not prove** that Nav2 is ready.

Nav2's lifecycle manager configures and activates all managed nodes in order. Any failed configure/activate transition can leave the entire stack unavailable. The currently installed custom Nav2 BT files are **the same** on the earlier release, seaport-audit and GLIM repair branches, so simply reverting those files is not evidence-based. The first configure failure still needs to be captured from the ROS launch log.

This recovery adds a reversible startup profile that uses Nav2 Jazzy's **installed stock behavior trees**. It does **not** use odometry from Gazebo ground truth, disable health gates, or send robot commands. If the lifecycle remains unconfigured, inspect Nav2's process logs rather than repeatedly rebuilding unrelated packages.

## What changes

- `nav2_bt_profile:=stock` (new Jazzy default) selects Nav2's installed `navigate_to_pose_w_replanning_and_recovery.xml` and `navigate_through_poses_w_replanning_and_recovery.xml`. Their absolute paths are validated before launch.
- `nav2_bt_profile:=custom` restores the original position-arrival and custom through-poses behavior trees without changing the controller/planner parameter YAML.
- `scripts/nav2_lifecycle_report.py` queries lifecycle services without applying any transitions, and reports both odometry streams, robot health and SLAM status.
- Existing custom trees, Nav2 costmap settings, motion interlock, dashboard, seaport collision model and spawn-height corrections stay intact.

**This is a candidate recovery, not a verified autonomous-navigation fix.** Validated behavior requires a real Jazzy/Gazebo run on the target laptop.

## Build only the changed ROS Python packages

Use a fresh shell; keep `niihan_ws`, `niihan_waypoint_review` and the already-compiled `niihan_glim_repair` worktrees.

```bash
cd ~/surveillance_project/niihan_ws
git fetch https://github.com/mayureshcode/niihan_ws.git   refs/heads/fix/nav2-jazzy-lifecycle-recovery-20261106
git worktree add --detach ../niihan_nav2_recovery FETCH_HEAD
cd ~/surveillance_project/niihan_nav2_recovery

source /opt/ros/jazzy/setup.bash
source ~/surveillance_project/niihan_ws/install/local_setup.bash
source ~/surveillance_project/niihan_waypoint_review/install/local_setup.bash
source ~/surveillance_project/niihan_glim_repair/install/local_setup.bash
source ~/surveillance_project/niihan_glim_repair/scripts/environment.sh

colcon build --symlink-install   --base-paths src/niihan_description src/niihan_slam   --packages-select niihan_description niihan_slam
```

This builds the two changed ROS Python packages only. It does not recompile GTSAM or GLIM.

## Run one controlled test

Stop the old simulation's launch process cleanly with Ctrl+C first, so there is only one Gazebo server, clock and dashboard. Open a fresh shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/surveillance_project/niihan_ws/install/local_setup.bash
source ~/surveillance_project/niihan_waypoint_review/install/local_setup.bash
source ~/surveillance_project/niihan_glim_repair/install/local_setup.bash
source ~/surveillance_project/niihan_nav2_recovery/install/local_setup.bash
source ~/surveillance_project/niihan_glim_repair/scripts/environment.sh

export ROS_DOMAIN_ID=188
export GZ_SIM_RESOURCE_PATH="$HOME/surveillance_project/gazebo_assets:${GZ_SIM_RESOURCE_PATH:-}"

ros2 pkg prefix niihan_description
ros2 pkg prefix niihan_slam
ros2 pkg prefix glim_ros
```

Expected paths:
- `niihan_nav2_recovery/install/niihan_description`
- `niihan_nav2_recovery/install/niihan_slam`
- `niihan_glim_repair/install/glim_ros`

If these differ, fix overlay ordering before launching.

```bash
read SPAWN_X SPAWN_Y SPAWN_Z < <(python3 - <<'PY'
import json
from pathlib import Path
path = Path.home() / "surveillance_project/gazebo_assets/site_robotics_seaport_spawnfix.json"
print(*json.loads(path.read_text())["spawn_xyz"])
PY
)

ros2 launch niihan_slam slam_simulation.launch.py   world:="$HOME/surveillance_project/gazebo_assets/site_robotics_seaport_spawnfix.sdf"   mode:=mapping headless:=false rviz:=false   spawn_x:="$SPAWN_X" spawn_y:="$SPAWN_Y" spawn_z:="$SPAWN_Z"   nav2_bt_profile:=stock environment_id:=sea-port-mapping-v1 seed:=42
```

Open `http://localhost:8080` for the existing dashboard.

## Diagnose (read-only)

In another terminal with the same sourced overlays and ROS_DOMAIN_ID:

```bash
python3 ~/surveillance_project/niihan_nav2_recovery/scripts/nav2_lifecycle_report.py

# If Nav2 is not ready, inspect the FIRST configure/activate failure in the
# main ROS launch terminal, not only the final "Nav2 unavailable" badge.
grep -RniE 'bt_navigator|Failed to configure|Failed to bring up|lifecycle_manager_navigation|Exception|error while loading'   ~/.ros/log/latest 2>/dev/null | tail -n 80
```

Expected acceptance (not yet measured): GLIM odometry samples, fresh `/odom`, all five dashboard-required Nav2 lifecycle nodes `active [3]`, action server ready, and a healthy, nonempty map where the rover is localized. The stock BT profile alone cannot guarantee these.

**Only after these pass** should a path through verified collision-free ground be tested. The seaport GLB collision proxies and site-to-map alignment have not been certified. Do not circumvent e-stop, health, TF or geofence checks.

## Regression controls

Use `nav2_bt_profile:=custom` only to compare against the former custom tree on the same installed environment. Changing profiles requires a normal restart of Nav2; it is not an online toggle.

Run the source checks on a prepared ROS workstation:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q   src/niihan_slam/test/test_nav2_launch_profile.py
```

References:
- Navigation2 Jazzy BT Navigator configuration: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_bt_navigator/
- Nav2 lifecycle manager: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/
