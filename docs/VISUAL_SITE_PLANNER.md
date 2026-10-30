# Visual site map — GLB top-down waypoint planner (draft)

This optional dashboard feature adds a third tab: **VISUAL SITE MAP**.
It does not change the 2D Nav2 planner, GLIM mapping, motion gateway,
geofence, or E-STOP. The GLB is a visual aid, never a navigation map.

## Use

1. From a cleanly built branch, launch the existing Site Robotics SLAM
   simulator as usual and open http://127.0.0.1:8080.
2. Open **VISUAL SITE MAP** and choose your local
   `industry_seaport_optimized.glb` (normally under
   `~/site_robotics_gazebo/models/site_robotics_port/meshes/`).
   The browser reads the file locally; it is not uploaded.
3. The model is rendered from above. Use scroll to zoom, drag to pan,
   and **FIT WORLD** to frame the entire site.
4. Adjust **X offset**, **Y offset**, **Yaw**, and **Scale** to align the
   displayed model with ROS `map` coordinates. The display assumes
   the GLB uses glTF's Y-up convention and converts it to ROS Z-up.
   Gazebo SDF model pose and GLB exporters may require calibration.
5. Verify alignment against **at least two distinct landmarks whose ROS
   map coordinates are known**, then check the explicit confirmation box.
   Changing any alignment field invalidates confirmation.
6. In a *stationary, non-running mission*, click an open aisle directly in
   the visual site map — the extra **ADD WAYPOINT** button is not needed
   on this tab. A known-free click sends the same `add_waypoint` command
   used by the 2D planner and appears in the mission table after server
   validation. Clicks do not start driving.
7. Unknown, occupied, or out-of-map clicks become **yellow/orange visual
   drafts**, never ROS goals. After mapping more space, select **SEND VALID
   DRAFTS**. Only points newly verified free are submitted. The others stay
   visually marked and are not part of the mission.
8. Use **FIND ROBOT** to zoom to the prominent cyan rover marker, and
   **ZOOM + / ZOOM −** to adjust site scale. `FIT WORLD` restores overview.
9. With healthy localization and Nav2, use **WRITE MISSION**,
   **AUTO** mode, and **START MISSION** through the original controls,
   only after verifying that the robot, waypoints, and actual ground align.

## Limitations and safety

- The GLB **does not create** a full map; grid coverage still depends on
  the SLAM pipeline and robot exploration. A 600×600 grid at 0.1 m/cell
  represents 60×60 m, irrespective of the visible GLB extent.
- Only **known free** cells in `/map` can become ROS mission targets.
  Unknown, occupied, and off-map cells are shown as unsent visual drafts. The clearance check
  uses 0.45 m around the picked coordinate; Nav2 remains the authority
  for route planning.
- Do not treat GLB geometry as proof that a point is traversable.
  Never click container roofs or visually clear but unobserved lanes.
- The visual planner requires the bundled Three.js r128 and a
  matching `GLTFLoader`, currently loaded from jsDelivr. If that
  CDN is unavailable, the 2D map and existing ROS mission controls
  continue to work unchanged; the visual tab displays an error.
- Draft waypoints are browser-session-only and disappear on reload; only
  successfully submitted points are saved in the live mission manager.
- Alignment settings and the selected GLB are browser-session-only:
  no model is automatically discovered or uploaded to the server.
- Verify the actual Gazebo SDF and GLB transforms before relying on
  calibration; errors could produce incorrect ROS coordinates.
- This is a source-level draft; no browser/WebGL + Gazebo integration
  test was executed. Do not merge into a working robot stack without
  checking model positioning, clicking, rejected cells, and normal
  mission cancellation in an isolated test worktree.

## Local isolated testing

```bash
cd ~/surveillance_project/niihan_ws
git fetch origin feat/glb-overhead-waypoint-planner
git worktree add ../niihan_ws_visual_test -b review/visual-map origin/feat/glb-overhead-waypoint-planner
```

The new worktree does not modify the dirty working directory or
include uncommitted seaport assets. Use the local GLB file input in
the browser; do not copy proprietary models into the Git repository
merely to preview them.
