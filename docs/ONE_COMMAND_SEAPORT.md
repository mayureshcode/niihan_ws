# One-command Site Robotics seaport startup (Ubuntu 24.04/Jazzy)

This branch consolidates the three recurrent issues into **one launch path**:

1. GLIM startup: reuse the **successfully source-built** GLIM/GTSAM pair from the independent `niihan_glim_repair` worktree. Check `ldd -r` and the ROS package prefix before launching.
2. Camera: install the dashboard implementation that **does not rotate the front camera another 180°**.
3. Robot placement: take the exact `spawn_xyz` from `~/surveillance_project/gazebo_assets/site_robotics_seaport_spawnfix.json`; forward Z to Gazebo.
4. Navigation: use native Jazzy Nav2 and installed stock behavior trees, with a configurable **15-second startup delay** to reduce races with GLIM/TF bootstrap; automatically report lifecycle states and odometry after startup. The old project BT can still be selected manually.

**This is a candidate end-to-end fix, not a demonstrated Nav2 recovery.** The last real device output showed GLIM odometry publishing, the health supervisor ready, and Nav2 lifecycle servers inactive / BT Navigator unconfigured. It did not identify the first Nav2 configure error. Delaying startup and selecting a stock tree may resolve a startup race or an invalid custom tree, but the exact underlying lifecycle failure is not yet proven. The launcher will print and save the remaining failure instead of silently reporting success.

## Install the unified branch without overwriting anything

On the HP OMEN, from a **new terminal**:

```bash
cd ~/surveillance_project/niihan_ws
git fetch https://github.com/mayureshcode/niihan_ws.git refs/heads/fix/unified-seaport-startup-20261106
git worktree add --detach ../niihan_unified FETCH_HEAD
cd ~/surveillance_project/niihan_unified

# Stop the old Gazebo/dashboard launch from its original terminal before this.
bash scripts/start_seaport.sh
```

That's the complete **build + launch** command. There are no manual source commands, package-path guesses, apt installs or GLIM rebuilds. The script:

- starts with a clean ROS environment and loads `/opt/ros/jazzy`, `niihan_ws`, then the compatible `niihan_glim_repair` overlay;
- refuses to run if the original workspace, compatible GLIM install, seaport SDF or spawn JSON is missing;
- refuses a second dashboard occupying port 8080;
- builds **only** `niihan_description`, `niihan_slam`, `niihan_dashboard` in `niihan_unified`, and confirms ROS is actually using these new versions;
- checks that the dashboard does not rotate incoming camera frames;
- executes static launch regressions and syntax checks;
- launches the exact spawn-fixed seaport with `nav2_bt_profile:=stock`, `nav2_start_delay:=15.0`, `mode:=mapping`, `ROS_DOMAIN_ID=188`, and Gazebo visible;
- prints the dashboard address `http://localhost:8080`, and runs a read-only Nav2 lifecycle/odom/health report automatically.

## Verification

Check `http://localhost:8080`. For **navigation success**, the subsequent diagnostic must report all dashboard-required nodes `active [3]`, `/navigate_to_pose action server ready: True`, actual `/glim_ros/odom` and `/odom` samples, and robot health ready. A publisher count alone is insufficient.

Every launch saves a console log in:

```text
~/surveillance_project/niihan_unified/.run/logs/seaport-YYYYMMDD-HHMMSS.log
```

If the dashboard still shows `NAV2: NOT AVAILABLE`, do **not** rebuild or reinstall anything again. The first lifecycle manager / configure / plugin error in that log is the next required evidence. Check it with:

```bash
grep -niE 'lifecycle_manager|bt_navigator|configure|exception|failed|error' \
  ~/surveillance_project/niihan_unified/.run/logs/seaport-*.log | tail -n 90
```

The automated report is read-only and never forces a lifecycle transition.

## Scope / safety

The robot is only authorized to move after fresh localization, Nav2 readiness, health permission and collision-safe waypoints. The launcher does **not** remove the emergency stop, watchdog, arbiter, map/TF requirements or obstacle checks. GLB alignment and world collisions still need independent validation; a visible seaport is not proof of safe autonomous navigation.

The navigation BT profile is reversible in the underlying launch (`nav2_bt_profile:=custom`); the unified one-command startup intentionally uses the standard Jazzy profile to reduce variables.

Earlier PRs #5 (GLIM ABI repair) and #6 (Nav2 stock BT/diagnostics) are included by ancestry; don't manually cherry-pick them or source multiple review worktrees into the unified launch.
