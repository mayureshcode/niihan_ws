# Site Robotics — Seaport Rover software architecture review (2026-10-31)

Status: **code-reviewed and patched; live Ubuntu/Gazebo validation pending.** This
is a deliberately isolated branch stacked on the unmerged simulation startup
isolation fix. It is **not** proof that the system is production/industry ready.

## System boundaries and data flow

```text
Gazebo Harmonic (one server, simulation /clock)
  |  sensor bridge (ros_gz_bridge)
  |-- RGB Camera  /niihan/sensors/panoramic/front/image_raw
  |                  -> Dashboard ROS bridge / recording tool -> engineer
  |-- IMU         /niihan/imu/data  ------------------------+
  |-- 3D LiDAR    /niihan/legacy/lidar/points               |
  |                  -> SensorAdapter ----------------------+-----> GLIM LIO
  |                        |                                |
  |                        +-- /niihan/sensors/lidar/points  |
  |                        +-- /scan -> HealthSupervisor/Nav2 costmaps
  |-- simulated GNSS -> RTK monitoring / alignment
  |
  + GLIM 3D LiDAR/IMU localization
          -> LIO TF filter -> map/odom/base_nav TF
          -> SLAM map bridge -> /odom, /map, /niihan/slam/map_cloud
                             -> Nav2 2D planning
                                    -> /niihan/cmd_vel/nav
 Dashboard WebSocket manual/mission commands -> ROS Dashboard Bridge
                                    -> /niihan/cmd_vel/manual
                     command_arbiter (manual > recovery > navigation)
                                    -> /niihan/cmd_vel
    HealthSupervisor (requires live LiDAR, /scan, /odom, IMU, drive healthy)
      -> /niihan/motion_permitted -> MotionGateway <--- E-STOP
                                    -> /niihan/drive/cmd_vel
                                    -> Gazebo drive relay
                                    -> simulated rover
```

The safety stop and health requirements are separate from the visual map.
The GLB map is **display/planning-only**. Its objects, colors and coordinate
labels are NOT certified collision geometry or a validated occupancy map.

## Confirmed code defects and patch status

| Severity | Finding | Change |
|---|---|---|
| High | With `external_stack=true`, Gazebo bridge remaps LiDAR from `/niihan/sensors/lidar/points` to `/niihan/legacy/lidar/points`, while the SensorAdapter previously used its default `/niihan/raw/lidar/points`. This can leave `/scan` without a publisher/data even though Gazebo has LiDAR. | This branch explicitly selects `cloud_topic=/niihan/legacy/lidar/points` in the high-level launch. |
| Medium | Previously the adapter silently dropped untransformable scans and exposed no source/TF conversion counters. | Added `/niihan/sensor_adapter/status` with source topic, received clouds, published scans, TF failures, latest ages and last error. No false scans are synthesized. |
| High | The original high-level launch silently ignored `drive_enabled` and `spawn_z`, with the gateway hard-coded enabled. | Already addressed by the **separate unmerged prerequisite** `fix/simulation-startup-isolation`, which is the base of this patch. |
| High | Multiple Gazebo servers can compete for `/clock`, leading to inconsistent sensor timing. | Prerequisite branch adds startup refusal when a known SDF Gazebo server is running; does not indiscriminately kill processes. |
| Medium | Large visual yard vs small default navigation map. | Prerequisite branch permits explicit `map_resolution` and `map_size_cells` (default 600 at 0.1m/cell). Changing this is NOT proof that any space has been scanned. |
| Medium | Map-frame coordinates are not the same as Gazebo spawn coordinates. An unverified GLB alignment can put waypoints in obstacles. | Browser visual planner is on an **independent PR**; must verify two landmarks/origin, scale and yaw before sending waypoints. Do not hard-code a 10m translation from a single observation. |

## Important open issues; NO green light for motion yet

1. **Live source and TF freshness:** `/niihan/legacy/lidar/points` must have
   a publisher and adapter status `published_scans` must increase. If
   `received_clouds` rises but `published_scans` does not, diagnose TF
   timestamp/frame mapping from `last_error`; do not bypass the supervisor.
2. **Slow Gazebo physics:** Earlier reported RTF near 0.08 can make cloud,
   odometry and camera wall-time rates low. Freshness timeouts are intentionally
   not relaxed in this patch. Optimize the world/collision bodies or diagnose
   the actual throughput before retrying autonomous motion.
3. **Motion proof:** Inspect `/niihan/health` and
   `/niihan/motion_gateway/status`, Nav2 lifecycle, localization continuity,
   and the real commanded-vs-observed speeds on a short straight route; test
   E-STOP, stale commands and side obstacles before increasing the speed.
4. **No end-to-end ROS/Gazebo execution in GitHub API:** Pure Python/AST
   regression tests only validate a code contract. Gazebo, GLIM, TF, WebGL and
   runtime QoS must still be tested on the Ubuntu machine.
5. **GLB changes:** New GLB can invalidate previously verified X/Y/yaw/scale.
   Refresh visual alignment and confirm SDF visual/collision placement; preserve
   consistent axes and container ID labels. Never reuse old calibration blindly.
6. **Camera deliverable:** Camera bridge topic is
   `/niihan/sensors/panoramic/front/image_raw`. Verify a live publisher/rate,
   then record direct camera images with
   `tools/record_site_camera.py` from the **visual planner feature branch**,
   or use `ros2 bag record` for original sensor fidelity. Keep simulation
   overview footage distinct from the rover's RGB camera output.

## Minimal non-driving validation

With the patched worktree built and sourced, one running Gazebo server,
`ROS_DOMAIN_ID=188`, and the robot stationary:

```bash
bash tools/seaport_preflight.sh
ros2 topic hz /niihan/legacy/lidar/points
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /niihan/sensor_adapter/status --once
ros2 topic echo /niihan/health --once
ros2 topic echo /niihan/motion_gateway/status --once
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

Expected: an active raw LiDAR publisher; increasing adapter
`received_clouds` and `published_scans`; latest ROS messages without
exceptions; `health.ready=true` with `faults=[]` sustained; Nav2 active.
`motion_gateway/status` may still report disabled or command stale; that is
correct for a parked robot. Do not interpret Nav2 READY alone as safe motion.

### ROS-independent regression

```bash
python3 -m pytest -q \
  src/niihan_bringup/test/test_contracts.py \
  src/niihan_bringup/test/test_drive_inhibition_diagnostics.py \
  src/niihan_slam/test/test_grid.py \
  src/niihan_slam/test/test_sensor_wiring.py
```

The static wiring test protects against reintroducing the wrong adapter
LiDAR topic. The existing pure-Python test suite protects gateway contracts
and grid sizing. No claim of passing live ROS tests is made here.

## Merge/order recommendation

1. Validate `fix/simulation-startup-isolation` (original draft PR #1).
2. Validate this stacked patch with a **single** running Gazebo server.
3. Review/merge the fixes into the active ROS integration branch after
   acceptance; keep the GLB and optional dashboard feature independently
   reviewable.
4. Re-run all checks after importing the user's revised seaport GLB/SDF.

Never run `git reset --hard`, `git clean -fd` or overwrite the user's local
untracked seaport assets to apply these changes.
