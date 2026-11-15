# Seaport site-containment safety candidate — NOT live validated
Branch: `fix/seaport-site-containment-20261115`
Based on: `feat/core-mou-home-status-alerts-20261114`.

## Incident
On 15 November 2026, the simulator robot drove off the site platform.
Operator confirmed the running source had `GridBased.allow_unknown: true`, global and local `cliff_layer.enabled: false`, and a forward LiDAR Collision Monitor StopZone of radius 0.6m. The site geofence requirement was declared but false by default; the launch did not opt in.

These are **risk factors**. They do not prove why the specific controller route reached the edge; the `/plan` bag and platform geometry have not been inspected.

## Changes in this safety candidate
1. `nav2_3d_jazzy.yaml`: Nav2 SmacPlanner2D `allow_unknown: false`. Unknown map cells may no longer be treated as traversable by that planner. This WILL reject legitimate goals until mapped free corridors exist.
2. `slam_simulation.launch.py`: For world or `environment_id` containing `seaport`, set dashboard `require_site_geofence: true`. The established `run.sh` command is unchanged. Other environments are unaffected.
3. Existing map-frame single GO polygon geofence, 0.45m inset margin, map keepout masks (local + global), pose freshness gate and software E-stop remain unchanged.
4. Existing `cliff_layer.enabled: false` stays unchanged. It must NOT be enabled until a real `/cliff_scan` detection source is verified.

These changes **only** restrict dashboard-initiated movement and SmacPlanner2D paths; they are NOT a hardware-independent containment guarantee and do not block every raw ROS command or force all robot movement to respect a platform edge.

## Mandatory manual release gate — do not skip
- Keep E-stop engaged and Gazebo paused until checked. Do not repeat the failed distant-goal mission.
- Confirm the real **map-frame site floor perimeter** using Gazebo ground-truth/landmarks and calibrated TF, not the old static robot in the GLB or visually guessed screen coordinates.
- Define an inward-offset GO polygon strictly inside supported floor geometry, preserving stopping-distance + localization margins, using the 2D map's **CREATE GEOFENCE** tool. This is a *site polygon*, not an outline of the entire 1200x1200 occupancy-grid canvas. Record its vertices as a reviewed file and verify agreement with Gazebo.
- When received, the dashboard must show `geofence.enabled=true`. Ensure `/niihan/geofence_mask` and `/niihan/geofence_filter_info` publish with the right map frame and geometry; inspect the actual local/global costmaps and test rejection of outside goals.
- **Install verified Gazebo boundary colliders** around platform edges for further simulation. The current seaport collision-preview SDF and GLB are not stored in this remote branch; they must be edited/tested from the approved local asset. Do not invent platform-boundary coordinates.
- Revalidate all safety paths: manual mode, AUTO goal, mission resume, stale localization, map coverage, collision monitor, emergency stop. Demonstrate no-goal execution when geofence missing and when path crosses boundary; evaluate stopping behavior close to edge at low speed after reviewing risk.
- If map-frame localization drifts relative to Gazebo or map labels water-side space free, pause and fix alignment/map source. Software keepout alone may then be unreliable.

## Safe apply instructions (NO blind git checkout / merge)
After **stopping the current simulation** and preserving local work, fetch the feature ref and inspect the three changed files:
```bash
cd ~/surveillance_project/niihan_ws
git status --short
git fetch origin refs/heads/fix/seaport-site-containment-20261115:refs/remotes/origin/fix/seaport-site-containment-20261115
git diff --stat origin/feat/core-mou-home-status-alerts-20261114..origin/fix/seaport-site-containment-20261115
```
Backup **before** applying `src/niihan_slam/config/nav2_3d_jazzy.yaml` and `src/niihan_slam/launch/slam_simulation.launch.py`, then copy each from the new ref using `git show REF:path > path`. Rebuild `niihan_slam`, source overlay, and launch only after syntax/pytest pass, using the **unchanged**:
```bash
cd ~/surveillance_project/niihan_ws
export ROS_DOMAIN_ID=188
./run.sh world:=site_robotics_seaport_collision_preview.sdf environment_id:=site-robotics-seaport-collision-v1 mode:=mapping spawn_x:=-35.0 spawn_y:=-10.0
```
With no site polygon, dashboard motion must be refused. That is **intentional**.
```bash
ros2 param get /planner_server GridBased.allow_unknown
ros2 param get /niihan_dashboard_node require_site_geofence
ros2 topic info /cliff_scan
```
Expected first two results: `false` and `true`. Cliff scan absence or empty output means do not enable cliff layer.

## Test plans after map-floor perimeter calibration
T00 Nav2 ready; geofence missing blocks command; known interior polygon accepted; outside polygon rejected; local/global costmaps show keepout; manual/auto and resume blocked as expected; robot collision stopped near edge without traversing forbidden area. Confirm no motor outputs after E-stop. T01 and T02 known-map lanes; T05 obstacle recovery; T08 TF/Gazebo position accuracy; T11 emergency-stopping. Record rosbag, screenshots and exact commit SHA. If any test fails, keep this safety PR draft.
