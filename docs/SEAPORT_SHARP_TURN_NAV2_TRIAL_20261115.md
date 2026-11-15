# Seaport Sharp-Turn Navigation Candidate — 15 November 2026

**STATUS: DRAFT, NOT LIVE VALIDATED. This is NOT an autonomous-safety guarantee.**

Baseline: `fix/nav2-route-preflight-visualization-20261115`. This candidate changes only Nav2 parameters and static tests. It **does not alter the GLB, Gazebo SDF, robot URDF, JS dashboard, waypoint coordinates, GLIM backend, teleop API, behavior tree, geofence, cliff detection or collision monitor**.

## Sources / rationale
- Official Nav2 Jazzy Rotation Shim Controller documentation: rotate safely toward the initial path heading before handing control to the primary path follower; see https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_rotation_shim_controller/.
- Official Nav2 Jazzy PoseProgressChecker: a skid-steer rover can show genuine progress during a slow 90-degree turn while XY stays almost stationary; see https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/controller_server_plugins/pose_progress_checker/.
- Official Nav2 Jazzy RPP parameters: lookahead and curvature-based regulation must be platform-tuned; see https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/.
- Comparable *architecture*, not directly copied tuning: `Tarif-T/Nav2_Smartwarehouse_project` (Jazzy + Harmonic + narrow aisles), `yaochongchow/warehouse-delivery-robot` (Nav2, BT and mission workflows).

## Proposed changes
| Parameter | Previous | Candidate | Why |
|---|---:|---:|---|
| FollowPath plugin | RegulatedPurePursuit | RotationShim wrapping RPP | Align to new path heading before forward tracking |
| rotation shim threshold | — | 0.80 rad | Only engage on major heading mismatch |
| rotation shim disengage | — | 0.40 rad | Avoid tiny correction oscillations |
| rotation shim speed | — | 0.40 rad/s | Within existing safety limit of 0.60 |
| rotation shim max acceleration | — | 0.80 rad/s² | Smooth startup |
| rotation shim simulate ahead | — | 1.5 s | Check collisions during in-place turning |
| rotation shim once | — | true | Avoid triggering repeatedly on every 2Hz replan |
| ProgressChecker | SimpleProgressChecker | PoseProgressChecker | Count rotation as legitimate progress |
| required movement angle | — | 0.35 rad | Allow valid skid-steer rotations |
| RPP lookahead | 0.60 m | 0.45 m | Improve fidelity near tight bends |
| RPP min curvature speed | 0.05 m/s | 0.12 m/s | Match documented valid range (>0.1) |
| RPP approach speed | 0.03 m/s | 0.05 m/s | More robust low-speed behavior |
| RPP heading-turn threshold | 0.25 rad | 0.45 rad | Reduce over-eager in-place corrections |

**Not changed:** `allow_unknown=false`, geofence, base footprint, collision monitor 0.6m StopZone, inflation 1.0m, RPP collision detection, SmacPlanner2D, current bounded recovery BT.

This proposal may help initial heading alignment and successful recognition of sharp-turn progress. It **will not solve** missing traversable lanes, a costmap with disconnected components, malformed SDF collision transforms, absent Gazebo platform edges, or collision-monitor deadlock. It may regress turn behavior; the trial must be A/B against the old stack.

## Installation (reversible, preserve local changes)
1. Stop `run.sh`, Gazebo, and mission, and ensure E-stop. Backup your local YAML.
2. Fetch `feat/seaport-sharp-turn-stability-20261115`. Inspect the diff. Copy **only** `src/niihan_slam/config/nav2_3d_jazzy.yaml` if your local version matches the expected baseline; otherwise cherry-pick the controller block manually.
3. Confirm `ros2 pkg prefix nav2_rotation_shim_controller` succeeds before launch; install a matching Jazzy binary package through your normal ROS distribution if it is absent.
4. `python3 -m pytest -q src/niihan_slam/test/test_sharp_turn_profile.py` after installing test file; then `colcon build --symlink-install --packages-select niihan_slam --parallel-workers 1` and source overlay.
5. Launch using **unchanged** validated command:
```bash
cd ~/surveillance_project/niihan_ws
export ROS_DOMAIN_ID=188
./run.sh world:=site_robotics_seaport_collision_preview.sdf environment_id:=site-robotics-seaport-collision-v1 mode:=mapping spawn_x:=-35.0 spawn_y:=-10.0
```
6. Keep autonomous drive OFF until a surveyed, safe platform GO polygon and Gazebo edge colliders are confirmed. Observe the Nav2 lifecycle and controller plugin initialization; a YAML parsing or plugin error is a **stop condition**.

## Required acceptance matrix — operator supervised, safely inside platform
- T00: Stable Nav2, TF, odometry, /scan, map-frame localization, local + global keepout masks, command and e-stop protection. No motion when e-stop active.
- T01: **90-degree turn** through a wide known-free lane; record start/end pose, yaw overshoot, deviation from /plan, accepted/rejected goal, controller error.
- T02: **S-bend and consecutive 90-degree turns**, 3 waypoints, safe speeds; no spin loops or collision stops without alert.
- T03: **180-degree change of heading** in clear space, verify rotation/shim and pose progress. Rotational motion counts as progress, but stalls still time out.
- T04: **Container aisle** with physically verified clearance above footprint plus margins, ensure collision monitor does not deadlock.
- T05: **Blocked aisle** with safe alternative route; bounded replan or fail-stop; never bypass keepout or explore unknown.
- T06: **Platform-edge limit**. No hazardous edge approach without physical barrier; demonstrate refusal using planning-only tasks at an out-of-bound waypoint.
- T07: Pause/resume, cancel, E-stop during turn, stale TF/wheel odometry. No continued motor command after stop.

**Proposed release targets** (must be measured, not claimed): zero collisions/falls, zero site-boundary breaches, 100% safe stop on emergency triggers; compare success rate and tracking error against baseline over repeated fixed-seed trials. Record bags of /plan, /tf, /odom, /niihan/wheel/odom, /cmd_vel_nav, /cmd_vel_smoothed, /cmd_vel and camera for review.

## Rollback
Copy the backed-up original YAML file back to `src/niihan_slam/config/nav2_3d_jazzy.yaml`, rebuild `niihan_slam`, source overlay and restart. Do not use `git reset --hard` or overwrite local SDF/GLBs.

## Limitations
Never label the project 'foolproof' or production-ready based on only source changes. Nav2 safety depends on accurate map, matched Gazebo/robot footprint, platform protection, sensors, TF and control chain.
