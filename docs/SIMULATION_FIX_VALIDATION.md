# Simulation reliability fix — isolated validation

This change is **not merged** into `feat/l2-3d-slam-release` and does not
include uncommitted local seaport files, external GLBs, or dashboard edits.

## Preserve the existing local workspace

Do **not** use `git reset --hard`, `git clean -fd`, `git add .`, or
`git pull` into a dirty working tree to try this change.

From the existing `niihan_ws` clone, inspect the patch first:

```bash
git fetch origin fix/simulation-startup-isolation
git diff --stat feat/l2-3d-slam-release..origin/fix/simulation-startup-isolation
git worktree add ../niihan_ws_fix_trial -b review/simulation-fix origin/fix/simulation-startup-isolation
```

The separate worktree has *committed* files only; private GLBs, local
`my_site.sdf`, generated collisions, and uncommitted changes stay in
the original directory.

## Scope

- Check for already-running Gazebo SDF servers before starting the stack.
- Forward `drive_enabled` and `spawn_z` launch parameters.
- Default driving **off** until explicitly enabled at launch.
- Disallow enabling a disabled motion gateway through a runtime parameter
  edit. Runtime disable stops motion and latches a stop until restart.
- Publish actual gateway and arbiter inhibition diagnostics.
- Add sensor age diagnostics without relaxing any sensor checks.
- Permit configurable occupancy-map dimensions; defaults are unchanged.

## Validation on Ubuntu 24.04 / ROS 2 Jazzy

Build and source the isolated worktree with the dependencies already installed
on the machine. Stop old Gazebo servers in their own launch terminals
first; do not kill an unknown simulator blindly.

```bash
cd ../niihan_ws_fix_trial
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select niihan_description niihan_bringup niihan_slam
source install/setup.bash
export ROS_DOMAIN_ID=188
```

Run a **stationary** smoke test first using a world committed to this branch:

```bash
ros2 launch niihan_slam slam_simulation.launch.py \
  world:=niihan_portable_site.sdf mode:=mapping \
  drive_enabled:=false headless:=true rviz:=false
```

From another terminal with the same workspace sourced:

```bash
ros2 param get /motion_gateway drive_enabled
ros2 topic echo /niihan/motion_gateway/status --once
ros2 topic echo /niihan/command_arbiter/status --once
ros2 topic echo /niihan/health --once
ros2 lifecycle get /bt_navigator
```

Expected for disabled drive: `enabled=false`, no forward motion,
and `inhibition_reason=drive_disabled`. `command_stale` when stationary
with driving enabled is expected until a fresh command arrives.

Test a runtime parameter enable attempt on a disabled gateway:

```bash
ros2 param set /motion_gateway drive_enabled true
```

It must be **rejected**, and `ros2 param get` must still return false.

For drive tests, start a fresh, single-server simulation with
`drive_enabled:=true`; verify healthy sensors, fresh TF and physical
clearance, then clear the E-stop and arm normally. Never bypass the gateway.
Check output on `/niihan/drive/cmd_vel` and physical motion separately.

For a larger site map, an *optional* example is:

```bash
# 1200 cells at 0.2 m/cell covers 240 m per axis
ros2 launch niihan_slam slam_simulation.launch.py \
  world:=niihan_portable_site.sdf \
  map_resolution:=0.2 map_size_cells:=1200 \
  drive_enabled:=false rviz:=false headless:=true
```

This is a map-capacity configuration, **not** evidence of complete
seaport coverage, correct mapping, or collision-safe navigation.

## Tests and limitations

```bash
python3 -m pytest -q \
  src/niihan_bringup/test/test_contracts.py \
  src/niihan_bringup/test/test_drive_inhibition_diagnostics.py \
  src/niihan_slam/test/test_grid.py
```

These pure-Python tests also run in the PR's GitHub Actions workflow when
available. They do **not** validate ROS 2 launch, Gazebo physics, GLIM,
TF continuity, live camera, or Nav2 motion.

Record RTF, camera/LiDAR rates, TF freshness, E-stop behavior, and
a verified short path before considering the patch ready to merge.
