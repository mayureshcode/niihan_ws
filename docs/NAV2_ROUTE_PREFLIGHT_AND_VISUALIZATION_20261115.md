# Seaport Route Gate and Live Nav2 Path — DRAFT / Not Gazebo Validated

## Incident facts
On 2026-11-15, map-frame waypoint start (-3.91, 9.16) to goal (-2.61, -0.52) returned Nav2 ComputePathToPose **208 (NO_VALID_PATH)**; a read-only connectivity check confirmed **NO POINT-CELL CONNECTION** within the final global costmap snapshot. A separate start (-12.46, 25.50) to goal (-10.23, 21.93) generated 40.72 m of path for 4.21 m direct distance, although that route and costmap snapshot were taken at different times. Earlier goal (-1.59, 2.03) was occupied in /map.

### This patch does not manufacture a path
The observed map/costmap disconnected free-space. Before physical patrol, revise the surveyed site GO boundary, SLAM obstacle evidence and collision model in the *local* seaport SDF; do not decrease safety thresholds or route blindly into unknown space. The SDF/GLB assets are not on this branch. This patch **prevents unsafe dispatch**, not fixes map connectivity.

## Changes
- Async Nav2 ComputePathToPose action executed for each dashboard-issued navigation target before calling NavigateToPose (including single-goal, multi-waypoint legs and return-home). On 206 occupied, 208 no path, timeout, wrong-frame path, unsafe geofence crossing, excessive detour, unavailable planner, E-stop or loss of readiness: **do not send movement goal**.
- Bounded route-detour gate: reject route length > max(12 m, 4 x straight-line distance); configurable `max_route_detour_ratio` default 4.0 on `/niihan_dashboard_node`. A detour guard is advisory for ambiguous routes, not a proof a route is safe or unsafe.
- Live Nav2 `/plan` subscription and plan snapshot via dashboard websocket `nav_plan` type. 2D view shows actual Nav2 path in cyan; rejected route in orange. Existing green waypoint-order line remains distinct. Planning telemetry has `PLANNING` state.
- Planning timeout: 10 s wall-clock safety abort for pending preflight. Nav2 SmacPlanner2D `max_planning_time` changed from 0.4 s to 2.0 s, **no changes** to footprint, obstacle inflation, collision monitor, geofence, `allow_unknown: false` or cliff detection.
- Cancelling or pausing a mission during preflight prevents late action-result callbacks from sending movement goals; cancelled preflights are discarded. The navigation goal is only sent after fresh mode, geofence, sensor health and TF checks.

## Local application (do not overwrite other frontend enhancements)
Prefer reviewing the 6-file diff against `fix/seaport-site-containment-20261115` and merging with local changes. The local dashboard likely has extra Satellite/Visual Site Map modifications. Never blanket checkout/reset all files.

Install `src/niihan_dashboard/niihan_dashboard/route_policy.py` and integrate the following files:
- `src/niihan_dashboard/niihan_dashboard/ros_bridge.py`
- `src/niihan_dashboard/niihan_dashboard/dashboard_node.py`
- `src/niihan_dashboard/web/js/viewer2d.js`
- `src/niihan_dashboard/web/js/dashboard.js`
- `src/niihan_slam/config/nav2_3d_jazzy.yaml`

Build: `colcon build --symlink-install --packages-select niihan_dashboard niihan_slam --parallel-workers 1`. Source overlay; verify the new tests in `src/niihan_dashboard/test/test_route_policy.py` and `src/niihan_dashboard/test/test_navigation_results.py`, then inspect normal dashboard and preflight status with motion gated.

## Verification gates
- Do not drive until manually verified Gazebo-edge barrier and real map-frame site GO polygon. Existing rectangle on map is not a surveyed platform boundary.
- With a known occupied waypoint: no NavigateToPose dispatched, message includes Nav2 206.
- With disconnected costmap (operator's prior route): no NavigateToPose dispatched, message includes Nav2 208.
- Excessively circuitous route: orange path preview, navigation inhibited.
- With known safe mapped aisle and verified geofence, route computation succeeds, cyan plan appears, movement only after planning.
- Pause/cancel/E-stop while planning -> no late nav goal. Mode change and localization loss also stop operation.
- Regression: existing Visual Site, Satellite Site, camera and websocket remain functional.

### Limitations
Planning results can become stale between preflight and motion; Nav2 still re-plans while driving. This is NOT a certified safety system and cannot prevent falls without actual environment boundary and cliff/ground support protection. No live ROS 2 Jazzy or Gazebo tests have been run from the assistant environment.
