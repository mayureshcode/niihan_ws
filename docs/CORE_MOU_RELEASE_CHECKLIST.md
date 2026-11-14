# Site Robotics surveillance UGV — core MOU submission checklist
**Status:** Release candidate, not validated client acceptance. **Date:** 2026-11-14.

## Intent
This branch holds the **robotics-software baseline** for the Site Robotics client, without tying MOU acceptance to optional 3D GLB Visual Site Map, Satellite Map or advanced visual zone editing. These UI features can be submitted separately after coordinate calibration. Preserve any existing locally edited SDF/GLB and dashboard work; this branch does not overwrite that work.

**Branch lineage:** `feat/updated-seaport-environment` → `fix/seaport-nav-spin-arrival-20261114` → this release branch.

## Must-have software acceptance (record evidence, not assertions)

| Core deliverable | State to prove | Evidence / limitation |
| --- | --- | --- |
| ROS 2 architecture, working launch and health | Start with established `run.sh` args, Nav2 lifecycle active, camera/map/LiDAR/IMU topics present | Runtime T00, process/log capture |
| GLIM SLAM / localization | Valid TF, accurate map registration, sensible pose drift | T08. Simulated ground truth, TF and GLIM comparison |
| Nav2 / mission waypoints | Single-goal success, 3+ waypoint patrol, arrival dwell, pause/resume, cancellation | T01/T02 videos + action status + pose/odom bag |
| Collision avoidance and recovery | Unexpected obstacle handled without collision, bounded retries, safe abort | T05. The Spin recovery action was removed from this release candidate's Jazzy BT; validate clear/wait/backup behavior |
| Safety / geofence / stop | E-stop, stale command, stale localization, wheel movement feedback, permitted single site-boundary geofence | T11 including boundary and outside-goal rejects. Validate global AND local Nav2 keepout filters |
| Single RGB camera | Live feed and recorded clip during motion | Topic/image rate and video evidence |
| Robot telemetry/API | Command and JSON payload examples; health and mission states; safe manual/auto transition | Command response logs + client schema reconciliation |
| Checkpoint dwell | Successful stopped arrival while GLIM reports residual drift, moving/stale wheel odom rejected | Unit test plus live mission |
| Battery/charging and rule-based alerts | Software schema and simulated-input tests for battery, link, navigation, motion alerts | **Not confirmed complete.** Hardware battery/charging state requires real controller inputs |
| Return-home / return-to-dock | Define and demonstrate simulated return-to-home/dock command and safe stop | **Not confirmed complete.** Physical docking/charging is hardware dependent |
| Documentation / handover | Tested SHA, known issues, test matrix, setup, API documentation and recorded evidence | Pending final sign-off |

### Keep software and hardware responsibilities separate
Physical motor/controller integration, real sensor mounting/calibration, charging electronics, wheel testing and real-world validation require Sidd/client/hardware. Software can test all applicable interfaces with simulated or mock data; label such data clearly and never imply physical acceptance.

## What is out of the core MOU release
- GLB Visual Site Map, automatic visual alignment and compass/heading UI.
- Satellite mission planning, GNSS imagery keys and datum UI beyond agreed localization requirement.
- Multiple NO-GO polygons, per-area speed zones, advanced zone persistence and editor (API request R6/R7 additional scope). Existing **single inclusion geofence + keepout mask** is within the baseline safety test.
- Multi-camera/WebRTC inventory, image/event capture extensions, stable map IDs/versioned retrieval, role/auth UI, UTC provenance and optional API extensions unless separately approved.
- Python / perception / AI-ML tasks P1–P10 belong to their assigned owners, not automatic ROS-team acceptance gates.

Reference: user-supplied `surveillance_robot_api_scope_summary_tables-1.pdf` (14 Nov 2026). The summary indicates signed MOU §§2–3 and API Guide v1.3, but also says the MOU references a separate Offering document not available for review; reconcile that original signed document before formal closure.

## Evidence before client branch/tag push
1. Preserve the local working site world/GLB and uncommitted changes. Do not use `git reset --hard` or a blind merge.
2. Test launch:
   ```bash
   cd ~/surveillance_project/niihan_ws
   export ROS_DOMAIN_ID=188
   ./run.sh world:=site_robotics_seaport_collision_preview.sdf environment_id:=site-robotics-seaport-collision-v1 mode:=mapping spawn_x:=-35.0 spawn_y:=-10.0
   ```
3. Tests: T00, T01, T02, T05, T08, T11; include safe geofence-boundary tests, manual control checks and fresh-wheel-arrival checks.
4. Verify client API requirements against documented v1.3; clearly mark unimplemented or hardware-dependent items.
5. Put video/bag evidence, changelog, API example requests/responses, known limitations and stable commit SHA in handover package.
6. Ask Sidd to review simulation evidence and physical-hardware dependencies; request written client acceptance before declaring final.

## Important release warning
A ROS node being `active`, a map/camera publisher existing, or a UI badge saying `NAV2: READY` is necessary but **not** a substitute for navigation, obstacle or safety acceptance. No code in this branch proves that a robot cannot enter the water without tested localization and validated boundary geometry.
