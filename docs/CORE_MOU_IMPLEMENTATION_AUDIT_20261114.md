# NIIHAN / Site Robotics — Core software submission audit
**Audit:** 14 November 2026 | **Candidate:** `release/site-robotics-core-mou-20261114` | **HEAD observed:** `35951d09b55faaa87e5b80ca5b058899cf8e3223`
**Verdict:** **Conditional / not ready for client acceptance**. Core navigation architecture exists; some original-scope software features remain open and the demonstrated November world is not packaged in the candidate.

## Evidence and limits
- Compared GitHub branch to `feat/updated-seaport-environment` using commit comparison (five changed files).
- Inspected code: `src/niihan_dashboard/niihan_dashboard/{ros_bridge,mission_manager,safety_manager,geofence_manager,dashboard_node}.py`; `src/niihan_bringup/niihan_bringup/{common_nodes,gazebo_adapter,hardware_nodes}.py`; `src/niihan_slam/{launch/slam_simulation.launch.py,niihan_slam/map_bridge.py,config/nav2_3d_jazzy.yaml,config/navigate_to_pose_position_jazzy.xml}`; `src/niihan_dashboard/test/test_navigation_results.py`; `run.sh`.
- Inspected current API scope basis: user-provided *Surveillance UGV | Client API & MOU Status* (14 Nov 2026) and *NIIHAN Dashboard API Guide v1.3* (3 Nov 2026).
- **Contract limitation:** Original signed MOU and its incorporated separate Offering document were **not directly available for line-by-line verification**. Scope rows below follow the provided reconciliation documents, not an independent legal determination.
- **Runtime limitation:** No direct access to the operator's Ubuntu ROS/Gazebo processes or uncommitted local files. November user logs showed Nav2 controller/planner/BT/collision monitor active, /map/camera/wheel topics published, and XY tolerance 0.10 m; a separate November mission reached WP1 and aborted WP2 (Nav2 status 6). These facts do **not** prove T02 pass.
- Client repo `appnetuk-maker/SITE_Navigate` was not accessible through the GitHub connection (404). Client-side actual branch content **cannot be verified** here.
- Candidate HEAD's GitHub status returned a `Vercel` success; this is **not** ROS 2, Nav2, Gazebo or hardware acceptance.

## What is actually new in the proposed push (vs existing seaport baseline)
| Status | File | Change | Risk/test |
|---|---|---|---|
| Proposed source change | `src/niihan_dashboard/niihan_dashboard/ros_bridge.py` | Arrival stop gate uses fresh wheel XY+yaw velocity (<=0.05 m/s, <=0.05 rad/s), not drifting GLIM twist. GLIM/TF pose and dwell remain. | Verify when odom is moving, stopped, stale and during mission progression; hardware wheel source must be validated separately. |
| Proposed source change | `src/niihan_dashboard/test/test_navigation_results.py` | Adds wheel motion, stale stream, drift and lateral checks. | Tests exist in source; no ROS CI result seen for HEAD. |
| Proposed config change | `src/niihan_slam/config/nav2_3d_jazzy.yaml` | Nav2 XY goal tolerance 0.02 -> 0.10 m. | User runtime confirms 0.10 m; validate same on release source. |
| Proposed config change | `src/niihan_slam/config/navigate_to_pose_position_jazzy.xml` | Removes blind `Spin` from BT recovery sequence; preserves costmap clear, Wait and BackUp. | Recovery can still abort; require T05 path/obstacle/blocked-route test. |
| Proposed documentation | `docs/CORE_MOU_RELEASE_CHECKLIST.md` | Candidate acceptance gates and scope exclusions. | Checklist is not test evidence. |

**Existing in baseline** (not new contributions of this PR): GLIM bridge/map, Nav2, mission state machine, pause/resume, manual mode, safety gateway, software e-stop, single inclusion geofence and costmap keepout, camera/pointcloud/map/WebSocket/GNSS telemetry, simulated drive adapter, map save and old seaport validation archive. Do not report these as new fixes.

## MOU software-related feature matrix
Legend: **IMPLEMENTED** = relevant source exists, **RUNTIME SEEN** = recent user terminal/screenshot, **OLDER TESTED** = historical repo evidence, **PARTIAL** = missing important function/test, **OPEN** = no verified implementation, **HW DEP** = requires physical integration.

| MOU/expected core function | Candidate evidence | Assessment | Needed for defensible sign-off |
|---|---|---|---|
| ROS 2 architecture, node lifecycle, launch | `run.sh`, `slam_simulation.launch.py`, active Nav2 nodes in operator log | **IMPLEMENTED / RUNTIME SEEN** | Frozen clone, dependency manifest, exact launch test on candidate commit. |
| Drive controller abstraction | `MotionGateway` clamps commands to default 0.30 m/s linear, 0.60 rad/s angular; `GazeboDrive` relay; STM32 hardware bridge still in source | **IMPLEMENTED IN SIM / HW DEP** | Record custom motor-controller contract and mock/physical compatibility; hardware integration is a separate dependency, not a software waiver. |
| LiDAR + IMU + SLAM/localization | `slam_map_bridge` converts GLIM odom/point clouds to /odom, /map, cloud; sensor adapter; saved-map paths | **IMPLEMENTED / RUNTIME SEEN / CALIBRATION OPEN** | T00/T08 with TF, map coverage and actual collision-preview world. |
| Nav2 planning and waypoint patrol | SmacPlanner2D, regulated pure pursuit, BT, `MissionManager`, dashboard `NavigateToPose` action | **IMPLEMENTED / OLDER TESTED / NEW TEST NEEDED** | Current 3-waypoint turn/return route with bags and 0 aborts. Recent WP2 aborted. |
| Obstacle avoidance and recovery | Obstacle/inflation layers, keepout filters, collision monitor, BT clear/Wait/BackUp | **IMPLEMENTED / EFFECTIVENESS UNCONFIRMED** | T05 obstacle insertion, no-contact evidence and correct abort/recovery. |
| Pause, resume, cancel, manual teleop | Dashboard `process_command`, mission manager; motion gateway watchguard | **IMPLEMENTED / OLDER TESTED** | Current pause-stop/rejoin and cancellation evidence; manual driving must stay bounded at physical output. |
| Checkpoint arrival and dwell/observe | Configured 1.0s stopped dwell, three distinct TF/RTK samples, RTK bounded corrections, wheel feedback code | **IMPLEMENTED / SOURCE TESTS / LIVE REGRESSION NEEDED** | Current test of WP1→WP2→WP3 with stable arrival, no false progress and fresh wheel odom. Capture/observe semantics need definition if the MOU demands an action. |
| Return-home/return-to-dock | No `return_to_dock` or dock-list command or proven docking state machine found in dashboard/mission components | **OPEN CORE MOU GAP** | Build/demo basic simulated home/dock goal, cancel/safety checks, success/failure API and evidence. Physical docking hardware separate. |
| Software E-stop, stale commands, watchdog, safety | SafetyManager, 50 ms dashboard loop, HealthSupervisor, MotionGateway, Nav2 collision monitor; software estop publishes zero/cancel | **IMPLEMENTED / LIVE SAFETY TEST OPEN** | T11 deliberate stop/stale sensor/manual watchdog; distinguish from physical motor-power E-stop. |
| GO boundary/geofence | GeofenceManager validates one polygon, 0.45m margin, occupancy-grid keepout mask; filters on global/local costmap, outside checks | **IMPLEMENTED BUT OFF UNTIL CONFIGURED** | Publish/validate a site polygon and boundary breach test. Geofence disabled returns permitted; default cannot be presented as unconditional water-edge protection. |
| One front RGB camera | ROS `/niihan/sensors/panoramic/front/image_raw`, dashboard JPEG stream; user screenshot/topic pub | **IMPLEMENTED / RUNTIME SEEN** | Short recorded camera clip and topic info with timestamp/frame provenance if requested; WebRTC/multi-camera NOT baseline. |
| Rule-based alerts (battery/link/nav/abnormal motion) | Health faults and status exist; no complete dedicated rule-alert API set found in dashboard stream | **PARTIAL / GAP** | Define and test explicit event rules, thresholds, freshness/severity/ack policy as MOU requires; do not confuse health status with full alerts. |
| Battery and charging telemetry | `get_telemetry_json` includes pose, velocity, GNSS, health, mission, etc.; no battery/charging values | **OPEN CORE MOU GAP** | Add nullable/stale-safe schema and simulation fixtures; actual battery charge data depends on hardware. |
| Robot-side client API | Two-way JSON WebSocket on 8081, HTTP dashboard 8080, telemetry/map/cloud/camera/model/joints, mission/stop/geofence commands | **IMPLEMENTED / PARTIAL CONTRACT** | Capture real payloads and failures against API v1.3; missing battery/alerts/dock; WebSocket has no auth/TLS (loopback only, proxy for production). |
| Gazebo validation, docs and handover | `docs/seaport_validation` archive and scripts; build/setup docs | **OLDER TESTED / LATEST RELEASE NOT VERIFIED** | Demonstrate actual client world from frozen SHA, attach evidence, release manifest, known limitations and client sign-off. |

### Historical evidence is useful, but not the latest release acceptance
`docs/seaport_validation/{README.md,summary.json,results.csv,audit_checks.json}` reports **7/7 straight-route seaport trials and 21/21 waypoints completed on 20 Oct 2026**, with final max horizontal error ~4.89 cm **after ideal simulated GPS-assisted approach** and seven 3D map saves. The README explicitly reports earlier turn/return trials **3/7**. These are historical results, on a different world/configuration, not a proven Nov 14 candidate pass.

`docs/RELEASE_VALIDATION.md` references three repeated simulations (October 10) of a different flat-site/earlier source, plus incomplete later safety/obstacle/localization acceptance. Do not conflate those data with the November seaport collision world.

## Critical release blockers (ranked)
| Priority | Finding | Evidence | Closure |
|---|---|---|---|
| **P0** | The specific live world `site_robotics_seaport_collision_preview.sdf` is absent from candidate remote `src/niihan_description/worlds`. New `site_robotics_port` GLB and mesh assets are also absent from remote models tree. | Inspected GitHub directory listing, which contains older `seaport_01..07.sdf` and construction-site assets but not latest site world. | Commit/reconcile approved SDF, meshes, resources or provide a verified asset package with manifest and rights. Test clone in a fresh workspace. |
| **P0** | Return-to-dock software function not evidenced | Dashboard command set and MissionManager lack dock workflow. | Implement basic simulated return-home/dock, or get explicit signed scope resolution. |
| **P0** | Full alert rules and battery/charging API absent/unverified | API v1.3 §4 explicitly lists no battery telemetry; audited dashboard lacks fields/events. | Implement software interface, tests and simulated/stale/unknown handling; real inputs remain hardware-dependent. |
| **P0** | No current 14-Nov navigation+obstacle+geofence acceptance run for candidate | User logs show Nav2 active but recent WP2 aborted; Vercel-only CI status. | Execute T00/T01/T02/T05/T08/T11, publish evidence with candidate SHA. |
| **P1** | Safety boundary is disabled unless configured; unlimited visual map extent not proof of permitted route | `GeofenceManager.is_robot_inside` returns True when disabled; `keepout_mask` depends on live /map extent; cliff acceptance unverified. | Establish site boundary and map coverage, verify local/global filters and stop margin; no unguarded waterside mission. |
| **P1** | Hardware bridge still references STM32 / ZLAC8015D; client changed hardware | `hardware_nodes.py` has `Stm32Bridge`, marked requires verified MCU integration. | Document mock-versus-production adapter and what Sidd/client must validate; don't claim physical commissioning. |
| **P1** | No production WebSocket security | `dashboard_node.py` binds to localhost by default; no app-side auth/TLS; guide requires secured proxy. | Include deployment/network boundary in handover, no untrusted exposure. |
| **P1** | Latest code tests not independently shown passing | `test_navigation_results.py` exists, but only Vercel commit status observed. | Run ROS pytest/colcon tests, capture report. |
| **P1** | Client repository contents not inspected | `appnetuk-maker/SITE_Navigate` returned 404 to connected GitHub reader. | Verify remote permissions, destination, compare test SHA on client's actual repo before push. |

## Enhancements over or alongside MOU — credit without adding acceptance burdens
- GLIM LiDAR/IMU SLAM + planar map projection, map checkpoint/autosave and launch hardening exceed a minimal navigation proof.
- Nav2 BT with local/global costmap filters, collision monitoring, bounded recovery/stop verification; these **enhance** safe navigation if live tests pass (removing Spin is a trade-off, not guaranteed improvement).
- GNSS/RTK metrics, dedicated alignment path, restricted simulated RTK, and optional bounded correction exceed generic location telemetry; *do not claim real RTK lock in Gazebo*.
- Dashboard command/mission API, cached camera/cloud/model, supervised telemetry, waypoint ordering and retry/cancellation races add useful integration capability.
- Original inclusion geofence + costmap keepout is a meaningful safety improvement; multiple excluded NO-GO polygons and visual zone editor are additional request R6, **not required for core MOU unless client agrees**.
- GLB waypoint planner, seaport collision-preview environment work, Satellite Map and 3D visual dashboard are optional extras, and **not committed on this core-release branch**. They should not block original contractual software closure.

## Release decision
**Not yet green for final client acceptance.** The branch is a plausible core software candidate; it is *not* the same complete asset set that the operator recently demonstrated and still has in-scope software gaps. Keep PR #14 draft until P0 items close or the client explicitly signs a documented exception.

### Minimal final handover package
1. Immutable commit/tag + known-good seaport world/meshes/hash/license manifest + exact Ubuntu 24.04 ROS Jazzy/Gazebo Harmonic launch.
2. Test matrix T00, T01, T02, T05, T08, T11 with timestamps, command logs, videos and measured pass/fail.
3. Captured API v1.3 JSON request/response examples, alert battery/charging availability contracts, return-home/dock commands.
4. Safety evidence for e-stop, stale controls, geofence on/off, blocked-goal, reduced-speed limits.
5. Architecture/source mapping, documented optional features, excluded hardware responsibilities and outstanding limitations.
6. Sidd review and written client acceptance before declaring release complete.
