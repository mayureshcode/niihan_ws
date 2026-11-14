# Satellite GNSS Mission Planner (experimental)

An optional **SATELLITE MAP** tab in NIIHAN GCS. It does not change the
existing 3D SLAM, 2D NAV MAP, VISUAL SITE MAP, navigation nodes, run.sh,
motion gateway, geofencing, Nav2, or E-STOP.

## What works

- Interactive street tiles (OpenStreetMap). Switch to MapTiler satellite
  imagery by entering **your own** MapTiler key in the UI and clicking
  SATELLITE. No key is committed, uploaded to ROS, or persisted by this code.
  The key is included in MapTiler browser tile requests; restrict it to your
  dashboard's permitted web origins. Internet access required.
- A live GNSS robot marker from the existing /niihan/gnss/fix telemetry and
  existing dashboard WebSocket. Stale/invalid/0,0 fix does not locate the robot.
- Calibrated WGS84 lat/lon to ROS map XY conversion using a site datum plus
  yaw: yaw is the *counter-clockwise* bearing of ROS map +X from geographic east.
  For example, yaw 0 means map X east/map Y north. Supports sites up to 5 km
  from the origin. This local tangent approximation is NOT for long routes.
- Click satellite/street imagery to create orange unsent drafts.
- Explicit confirmation of georeferencing and SEND VALID GPS DRAFTS converts
  drafts to ROS XY and uses the same add_waypoint command as the 2D planner.
  A 0.45 m clearance square must be known-free on /map (occupancy <65).
  All invalid, unknown, occupied, or out-of-grid drafts remain unsent.
- Current mission waypoints appear as green satellite markers when calibrated.
  Existing WRITE MISSION / AUTO / START MISSION controls remain separate.

## Use safely

1. Start existing simulation with the CONFIRMED working local command:
   ./run.sh world:=site_robotics_seaport_collision_preview.sdf
   environment_id:=site-robotics-seaport-collision-v1 mode:=mapping
   spawn_x:=-35.0 spawn_y:=-10.0
   (Set ROS_DOMAIN_ID=188 beforehand.) **No launch files changed.**
2. Build the dashboard overlay using your normal colcon workflow, then refresh
   http://localhost:8080 with cache bypass. Check SATELLITE MAP.
3. Optional: obtain a MapTiler satellite API key and enter it in the password
   input; click SATELLITE. Street imagery still works without a key.
4. For real deployment, initialize NavSatFix with real coordinates and a valid
   GNSS lock. Simulated GNSS currently reporting (0,0) **cannot** position
   this map: georeference Gazebo first.
5. Enter a surveyed/reference lat/lon for a known ROS XY location and the
   map yaw. With a fresh GNSS fix, USE ROBOT AS ANCHOR can populate one
   correspondence, but it does NOT determine yaw.
6. Independently verify at least one *second separated fixed landmark* in
   both the satellite image and ROS map. Tick the explicit confirmation box.
7. Click the map to add pending GPS draft pins. Click SEND VALID GPS DRAFTS.
   Only mapped free goals are sent to the mission manager; no motion occurs.
8. Check the mission table and command responses. Write and start the mission
   using the original safety-gated dashboard actions.

## Safety boundaries / limitations

- Satellite pixels are not a traversability map. The ROS /map occupancy grid
  remains mandatory; **Nav2** is authoritative for feasible paths.
- Geographic calibration is manual (one datum + yaw). A true georeferenced
  SLAM/TF alignment should be validated with surveyed landmarks / navsat
  transforms before operation. Angular misalignment can be hazardous.
- The browser rejects physical goal submission without a *fresh RTK fix*,
  valid ROS localization, healthy Nav2, no E-stop, editable mission and a
  GNSS-vs-ROS position agreement within 3 m. The simulation flag permits
  valid georeferenced simulated GNSS without RTK.
- These are **UI submission gates**, not a replacement for a server-side
  authoritative geospatial safety contract. Do not enable unattended physical
  satellite missions before adding server-side GNSS/datum verification,
  signed calibration/config management, accurate robot footprint clearance,
  path/geofence validation, and physical testing.
- Draft pins, datum and map key are in-memory only and reset on page refresh.
- Timestamps and the existing GNSS freshness contract are supplied by
  ros_bridge.py. Confirm receiver accuracy, heading, covariance and transform
  calibration for each site. GNSS multipath is significant near containers.
- MapLibre GL JS is loaded from a pinned CDN; if unavailable, other views
  remain functional. Use a licensed offline tile package to support offline
  deployment. Do NOT bulk-download OSM's public tile service.
- No satellite API token is included. MapTiler terms/attribution and image
  permissions are governed by the selected imagery provider.
- No live ROS/Gazebo/browser test performed on this remote code branch.

## Tests

    node src/niihan_dashboard/test/test_satellite_planner.js
    node src/niihan_dashboard/test/test_site_planner_drafts.js

The new tab reuses existing WebSocket telemetry (including gnss) and sends
existing add_waypoint commands; no new ROS command type is introduced.
