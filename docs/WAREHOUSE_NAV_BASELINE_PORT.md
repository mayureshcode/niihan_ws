# Seaport navigation: warehouse baseline comparison and targeted fix

**Status:** Proposed change; not yet validated in a running seaport simulation. Do not describe this as a confirmed sharp-turn fix.

## What the GitHub comparison establishes

Reference: `mayureshcode/warehouse-surveillance-ugv`, branch `warehouse/main`.
Target: `mayureshcode/niihan_ws`, branch `feat/updated-seaport-environment`.

The two committed Nav2 Jazzy profiles are **identical** after project-name substitution:

- `src/warehouse_slam/config/nav2_3d_jazzy.yaml` ↔ `src/niihan_slam/config/nav2_3d_jazzy.yaml`
- `src/warehouse_description/config/nav2_params.yaml` ↔ `src/niihan_description/config/nav2_params.yaml`

The Nav2 core launch sequences are also the same apart from the project namespace.
Blindly copying the warehouse Nav2 YAML cannot address a seaport-specific cornering problem.

The verified functional difference lies in dashboard waypoint-arrival verification:
warehouse already checks **fresh, progressing simulated wheel odometry** for stopped-state confirmation, while the prior seaport implementation relied on `/odom`, which may report small nonzero velocities from SLAM. The fix uses `/niihan/wheel/odom` in simulated missions, retains the existing SLAM/RTK position tolerance and dwell checks, rejects stale or frozen wheel messages, and keeps existing velocity verification outside simulation. An arrival timeout now includes numerical XYZ/velocity diagnostics.

The seaport's external Gazebo bridge remaps `/odom` → `/niihan/wheel/odom` when `external_stack:=true`. **Verify the live topic before testing**; missing wheel odometry will correctly prevent a false positive arrival verification.

## Safe acceptance procedure (same robot/world/config)

1. Preserve current local edits to the seaport site planner and world geometry. Run this branch in an isolated worktree or commit/stash existing edits before integrating. Do not overwrite the seaport environment, robot Xacros or API.
2. Build only the modified dashboard package. Use the same `ROS_DOMAIN_ID` and validated mapping launch used for prior seaport tests.
3. Readiness: `ros2 topic hz /niihan/wheel/odom`; `ros2 topic hz /scan`; `ros2 lifecycle get /controller_server`; `ros2 lifecycle get /planner_server`; `ros2 lifecycle get /bt_navigator`.
4. **T01**: Single goal on a straight, clear aisle. Verify Nav2 success and dashboard arrival after `1 s` stopped dwell.
5. **T02**: Mission with at least six operator-defined waypoints. Confirm each waypoint reaches `COMPLETED` and overall status finishes.
6. **T03**: Sharp 90° corner. Compare actual route, requested command `/cmd_vel_nav`, safety-gated output `/niihan/cmd_vel/nav`, costmap, and robot orientation. Record path planning failure separately from controller inability to follow a valid path.
7. **T04**: Multi-turn/zig-zag route in a clear, known-free area. Watch for oscillation, path clipping, insufficient turning clearance, and stale TF.
8. **T05**: Static obstacle on patrol route. Confirm it is present in `/scan` and the local costmap; do not declare obstacle avoidance safe unless physical collision and the monitor response are validated.
9. **T06**: Stationary rover with nonzero SLAM-derived velocity. Verify fresh `/niihan/wheel/odom` allows arrival confirmation only when within the existing positional tolerance and only after the required dwell; verify stale wheel messages do **not** pass.
10. Repeat the initial six-waypoint route to check for regressions. Retain ROS logs, path/costmap snapshots and a short screen recording.

## Investigate sharp turns independently

If T03/T04 fail, capture evidence before tuning:
- Compare geometry and collision layers in the seaport world with the warehouse world; their structures are different.
- Check the Nav2 local costmap occupancy and inflation relative to **actual robot footprint** and container edges.
- Verify `map → odom → base_nav` TF freshness and that `/scan` is not empty/stale.
- Inspect local controller angular velocity limits, smoothing, acceleration and lookahead against the robot's skid-steer capabilities.
- Distinguish (a) global path not found, (b) valid path but controller unable to track, (c) collision monitor suppressing motion, (d) robot physically contacting obstacles, (e) Nav2 success followed by dashboard verification failure.
- Make controller/inflation changes only with before-and-after evidence and a reversible parameter diff.

## Scope and caveats

This branch deliberately does **not** replace seaport Nav2 configuration, world meshes, robot, mission planner, GLIM, safety system, API, or dashboard frontend. The patch does not change the independent motion gateway and does not guarantee that 90° corners or complex trajectories will work; it targets a verified **arrival-verification false failure**.

If the wheel-odom source is unavailable, fix the Gazebo bridge or sensor adapter rather than weakening the stale-data fail-closed check. For a physical rover, verify stopped speed from the appropriate trustworthy wheel/motor feedback source.
