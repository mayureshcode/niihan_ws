# Ubuntu 24.04 / ROS Jazzy — GLIM ABI recovery

## Confirmed failure

The simulated seaport dashboard can show a live camera while navigation remains unavailable when the GLIM executable fails at process startup. The observed failure sequence was:

- \`libgtsam_points.so.1 => not found\` and \`libgtsam.so.4 => not found\`.
- After installing apt libraries, \`libglim.so: undefined symbol: _ZTVN5gtsam28PreintegratedImuMeasurementsE\`.
- No \`/glim_ros/odom\` publisher, no usable \`/odom\` data, no \`odom -> base_footprint\` TF; health inhibits drive and Nav2 cannot navigate.

The missing symbol is the vtable for GTSAM's \`PreintegratedImuMeasurements\`. The apt GTSAM runtime and older compiled GLIM were not a validated ABI pair. Creating symlinks to differently versioned \`.so\` files will not repair the underlying compatibility.

## Recovery without overwriting the original working workspace

Run **from a fresh shell**. Keep the original \`~/surveillance_project/niihan_ws\` and any local world assets unchanged. Use a separate checkout of this recovery branch:

\`\`\`bash
cd ~/surveillance_project/niihan_ws
git fetch https://github.com/mayureshcode/niihan_ws.git \
  refs/heads/fix/jazzy-glim-abi-repair-20261104
git worktree add --detach ../niihan_glim_repair FETCH_HEAD
cd ../niihan_glim_repair
bash scripts/repair_glim_abi.sh
\`\`\`

The script uses the existing project's **pinned** GTSAM and \`gtsam_points\` source revisions, builds both in \`.deps/jazzy/install\`, and compiles \`glim\` + \`glim_ros\` from the pinned submodules in this new worktree. It checks \`ldd -r\` for missing libraries and unresolved symbols and rejects linking against GTSAM from a different prefix. Building native libraries requires available build prerequisites, storage, and compiler resources. If it fails, keep the output and do not run a broken executable.

It does **not** delete the original workspace's \`build\`, \`install\`, \`log\`, maps, GLB, Gazebo world, or camera data. It does not run sudo or modify system packages.

## Test GLIM in the currently running simulation

After a successful repair, open a **new** terminal:

\`\`\`bash
source /opt/ros/jazzy/setup.bash
source ~/surveillance_project/niihan_ws/install/local_setup.bash
source ~/surveillance_project/niihan_glim_repair/install/local_setup.bash
source ~/surveillance_project/niihan_glim_repair/scripts/environment.sh
export ROS_DOMAIN_ID=188

ros2 pkg prefix glim_ros
ldd -r ~/surveillance_project/niihan_glim_repair/install/glim_ros/lib/glim_ros/glim_rosnode \
  | grep -E 'gtsam|libglim|not found|undefined symbol'
\`\`\`

The prefix MUST resolve to \`niihan_glim_repair\`. The GTSAM libraries must resolve from that worktree's \`.deps/jazzy/install/lib\`; there should be no \`not found\` or \`undefined symbol\` entries. If the prefix is from the original workspace, stop and fix the overlay order.

When the original launch is still active and its GLIM has exited, the new GLIM process can be started *alone* for a diagnosis. Use the existing generated GLIM configuration:

\`\`\`bash
CFG="$(ls -td /tmp/niihan-glim-* 2>/dev/null | head -1)"
test -f "$CFG/config_ros.json" || { echo "No GLIM config"; exit 1; }

ros2 run glim_ros glim_rosnode --ros-args \
  -r __node:=glim_ros \
  -p use_sim_time:=true \
  -p config_path:="$CFG" \
  -r /tf:=/niihan/lio/tf
\`\`\`

Do not start two GLIM nodes. This standalone test must NOT be taken as proof of successful mapping/navigation.

In another terminal with the same ROS environment:

\`\`\`bash
ros2 topic info /glim_ros/odom
timeout 10 ros2 topic echo --once /glim_ros/odom
timeout 10 ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo --once /niihan/slam/status
ros2 topic echo --once /niihan/health
\`\`\`

The repair is complete only if live GLIM odometry and TF are available, \`/odom\` is fresh, SLAM map points increase, the health supervisor is ready, and Nav2 lifecycle states are active. The separate seaport world still needs verified obstacle collisions and navigation acceptance. Never bypass the motion permission or e-stop for a demonstration.

## Full graphical relaunch

When the new GLIM has passed the live check, end the old ROS launch normally. In a fresh terminal, source the original workspace followed by the new repair workspace and **then** \`scripts/environment.sh\` as above. Use the same proven seaport spawn-fixed world path and the saved \`spawn_xyz\` JSON from the earlier setup. This recovery branch also forwards \`spawn_z\`, keeps the successful one-camera dashboard correction, and avoids the LiDAR gravity-alignment dependency during startup. Keep \`headless:=false\`.

**Notes:** A matching GLIM build and correct shared-library resolution are necessary, not sufficient. The repair script does not claim that SLAM, RViz, Nav2 or a whole-site patrol was tested on the target GPU. The original LiDAR/map coverage (60 m × 60 m occupancy projection) also needs separate improvement before surveying the entire ~101 m × 101 m seaport.
