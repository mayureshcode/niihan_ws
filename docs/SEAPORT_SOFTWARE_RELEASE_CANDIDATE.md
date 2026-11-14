# Site Robotics seaport — software release-candidate gate (14 Nov 2026)

STATUS: **PREPARATION ONLY. NOT CLIENT-ACCEPTED OR RELEASED.**

## Scope and evidence
Reference: `surveillance_robot_api_scope_summary_tables-1.pdf` supplied in review chat, 14 Nov 2026. The source calls its statuses *reported/documented*, not independently verified, and notes that the MOU incorporates an unavailable Offering document. Do not represent this document as a substitute for the signed MOU/Offering.

## Critical integration gates — MUST resolve before client push

### RC-01 GLB <-> ROS frame calibration (BLOCKED)
- GLB renders plausibly at UI yaw 270°, **not proof** of map-frame alignment.
- Site geometry stays in Gazebo/world frame. The static robot model baked into GLB is an *old visual object*; never use it as live robot location.
- ROS `map -> base_footprint` and the live dashboard `pose` are already used for the cyan marker. Identify whether the apparent old robot is the GLB-baked mesh; distinguish the two.
- Obtain **two non-collinear static landmark correspondences** between the GLB and Gazebo/ROS map. Compare translation, yaw, axis orientation, scale and SDF visual/link/model transforms. Do not calibrate against an old robot spawn point.
- Verify rendering against 3rd landmark and the real Gazebo robot at **two different poses** (translation + 90° controlled turn). Establish that cyan marker heading +X corresponds to true robot forward direction, not a fixed visual mesh orientation.
- Only after verified calibration, persist a versioned (environment_id + GLB hash) alignment record. Reject stale records for different GLBs/world revisions.
- Waypoint acceptance outside known map coverage may be used for **planning drafts**, but MUST NOT be treated as verified navigability or automatically executed. Server geofence and Nav2 costmaps remain authoritative.

### RC-02 GO/NO-GO zones (PARTIALLY BUILT)
- **Existing**: single polygon inclusion geofence; map-frame validation; footprint margin; Nav2 keepout mask in global/local costmaps; dashboard E-STOP on boundary breach; geofence set/clear API.
- **Requested**: one GO site-boundary polygon **plus multiple NO-GO polygons**, e.g., waterside perimeter, containers, cranes. This is R6 additional scope according to scope summary; obtain client approval.
- Implement shared versioned zone model and validated polygons (reject self-intersections, NaNs, empty/invalid vertices and overlaps invalidating start pose); no-go point containment and edge inflation; combine masks in map frame; publish on both global/local costmaps; render and edit both zone types in 2D and aligned GLB planner.
- Motion start and goal checks must reject any path/robot outside GO or within NO-GO. On boundary breach or stale localization, cancel and issue safety stop; exercise separate manual and automatic tests.
- **Do not mark protection proven** until run-time tests of both costmaps and physical stopping distance. A software geofence alone cannot guarantee no water entry.

### RC-03 Nav2 and arrival (SOURCE CHANGES AVAILABLE, LIVE VALIDATION REQUIRED)
- Existing PR #11 (`fix/seaport-nav-spin-arrival-20261114`) removes blind Spin recovery from Jazzy BT, sets 0.10 m goal tolerance, and verifies stop with fresh wheel odometry. **Source is not yet merged into this branch.**
- Nav2 was observed active with 0.10 m tolerance; that does not prove obstacle approach, recovery, waypoint or geofence acceptance.
- Test: T00 sensors/TF/lifecycle, T01 single goal, T02 patrol and dwell, T05 unexpected obstacle, T08 localization, T11 E-STOP/recovery. Record bag + screenshot/video + PASS/FAIL per test.

### RC-04 Preserve seaport assets and dashboard
- The live local workspace has uncommitted client world/GLB and satellite/visual dashboard edits not available together in GitHub branch history. A repository reset, blind merge or replacement checkout would lose work.
- Start with read-only `git status --short`, `git diff --stat`, and back up source files. Merge selectively in a separate worktree after committing/saving reviewed local changes.
- Keep verified `./run.sh world:=site_robotics_seaport_collision_preview.sdf environment_id:=site-robotics-seaport-collision-v1 mode:=mapping spawn_x:=-35.0 spawn_y:=-10.0` unchanged.
- After full stack launch, verify dashboard satellite/visual tabs, GLB load, waypoint click coordinate match, map, camera, RTK status, Nav2 lifecycle, e-stop, go/no-go.

## SOFTWARE-ONLY MOU work and closure targets
| Area | What can be completed in software/simulation | Notes |
|---|---|---|
| ROS 2 architecture/interfaces | Wiring, mock-vs-live adapters, launch, lifecycle and API contract | Real motor/controller testing depends on client/Sidd. |
| SLAM/localization | Mapping/localization, TF audit, LiDAR+IMU integration, drift and loop checks, saved-map regression | Sensor calibration on real rover needs hardware. |
| Nav2 patrol | Single/multi-goal, route planner, checkpoint dwell, pause/resume, safe rejoin | Formal verification logs still needed. |
| Obstacle avoidance/recovery | Static/dynamic collision trials, blocked-route response, safety limits | Physical validation separate. |
| Mission control/teleop | Commands, modes, cancellation, API validation and health gate | Keep mode and e-stop safety. |
| Return-to-dock | Dock API/control state machine, simulated dock target, return-home tests | Actual docking/charging interfaces need hardware. Core within MOU per scope summary. |
| Safety supervision | Stale command, stale sensor, geofence edge, stop/restart, operator e-stop | Physical emergency-stop validation separate. |
| One RGB camera | Stream, feed integrity, operator demo, optional recording | Multi-camera inventory/WebRTC extra. |
| Alerts | Rule-based battery/link/navigation/motion alerts with simulated inputs | Real battery/charge signals depend on integration. |
| Robot telemetry/API | WebSocket payload examples, mission, pose, GNSS, safety and simulated battery/charge fields | Do not fabricate actual charging state. |
| Gazebo/verification | Simulation scenarios, reproducible builds, test reports and video evidence | Client hardware acceptance later. |
| Documentation/release | Architecture, API payloads, test matrix, limitations, fixed commit SHA, handover package | Essential original scope. |

## Additional scope / do not silently include in MOU
- R2 serial/board identity; R3 multi-camera inventory; R4 timestamped camera/pose and WebRTC; R5 capture-event extension; R6 multiple no-go/site polygons; R7 per-area speed zones; R9 saved mission/geofence store; R10 stable map IDs/retrieval; R12 UTC provenance; R13 auth/roles/audit in external app/proxy. Clarify R11 GPS↔map datum and R8 multi-dock extension.
- P1–P10 perception/Python/Core API tasks **are not automatic robotics-software deliverables**; coordinate interface events only.
- Additional work to acknowledge: GLB visual planner, seaport GLB/SDF customization, collision-preview geometry, demo dashboard/API integration, GNSS/EKF diagnostics, robust Nav2 lifecycle/bringup.

## Acceptance gates for any final client push
1. Freeze site world + GLB hashes and map/environment IDs.
2. Verify GLB transform against two landmarks and moving robot facing.
3. Apply GO/NO-GO safety policy through global/local costmaps; record boundary and excluded-area tests.
4. Run T00/T01/T02/T05/T08/T11 and record logs/video, including manual and automatic stops.
5. Finish within-MOU software parts or mark hardware dependencies and incomplete items explicitly.
6. Produce API examples and versioned release notes; get reviewer/client sign-off.
7. Only after these gates, select a tested SHA and release/tag; never claim this preparation branch is a final release.
