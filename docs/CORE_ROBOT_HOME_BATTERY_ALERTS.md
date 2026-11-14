# Robot-side core software additions — return-home, battery integration and alerts

**Scope:** ROS 2 robotics software. **Status:** Source changes on feature branch; live Jazzy/Gazebo acceptance **not yet executed**. Does NOT include a physical docking/charging workflow or AI vision/perception.

## 1. What is newly provided

- `set_home`, `clear_home`, and `return_home` use the **existing** authenticated-by-network-boundary-only WebSocket on `ws://127.0.0.1:8081/`. These are JSON `action` commands, not REST routes.
- The stored home is **map-frame memory only** and disappears on restart. An explicit `set_home` is required every session. Never assume the old spawn pose is current.
- `return_home` uses the exact same Nav2 `NavigateToPose` action, health, mode, localization, stop and single-GO-geofence checks as `nav_goal`. It rejects an active/pending/paused mission; first cancel and wait for cancellation, then return.
- No new motor path, safety bypass, waypoint heading requirement, AI model or automatic charging.
- Battery from standard ROS `sensor_msgs/msg/BatteryState` on **`/niihan/battery/state`**. Client/Sidd supplies the publisher; no battery simulator is started by this patch.
- Transition-based robot-operational alerts in normal telemetry: navigation failure, loss of previously valid localization, E-stop, geofence breach, drive/sensor faults, stale camera, manual stream timeout, low/critical/stale battery, sustained motion stalled/uncommanded motion.
- Opt-in `require_site_geofence` ROS parameter: when set true, the WebSocket refuses manual joystick and autonomous navigation requests unless the existing single inclusion GO polygon has been configured; leave false for the historical demo/launch until a verified site polygon exists.

## 2. API commands (sent as JSON one at a time)

Capture the **current fresh map-frame pose** as home (safe, no movement):
```json
{"action":"set_home"}
```
Or assign a reviewed `map`-frame position, with required X and Y:
```json
{"action":"set_home","frame":"map","x":-3.0,"y":-8.0,"z":-0.54}
```
Clear home (idle navigation only):
```json
{"action":"clear_home"}
```
Return home **only after a cleared e-stop, healthy sensors, fresh localization, AUTO mode, no active mission, and a safe verified path**:
```json
{"action":"return_home"}
```
The reply is a `command_response` with success/message. Success means a **goal request was sent**, NOT that the robot arrived. Observe `telemetry.navigation.state` for `COMPLETED` and `telemetry.navigation.message` for `Home reached`; check pose/goal accuracy. If the home is outside a newly configured geofence, the goal is rejected.

**Not equivalent to autonomous dock alignment or charging:** No docking station localization, approach geometry, latching or battery charge coupling is implemented.

## 3. Battery contract

The client/Sidd supplies a ROS 2 publisher:
```text
Topic: /niihan/battery/state
Type:  sensor_msgs/msg/BatteryState
percentage: fraction 0.0..1.0, not 0..100
present: true when battery reading is real/known
power_supply_status: standard BatteryState status enum
voltage: optional valid nonnegative numeric reading
```
Example telemetry shape (illustrative; not live evidence):
```json
{
  "battery": {
    "available": true,
    "fresh": true,
    "percentage": 67.0,
    "charging": false,
    "status": "discharging",
    "voltage": 24.2,
    "source": "/niihan/battery/state",
    "simulation": false,
    "age_seconds": 0.2
  }
}
```
Before data arrives: `available=false`, `fresh=false`, `percentage=null`, `charging=null`, `status="unavailable"`. After five seconds without a message: `status="stale"` and values are **cleared to null**; the old number is NOT reused. Bad/NaN/out-of-range percentage is unavailable, never reported as zero or treated as a low battery. Low threshold <=20%; critical <=10%.

For an explicitly synthetic demonstration **in simulation only**, first run:
```bash
ros2 param set /niihan_dashboard_node battery_simulation true
ros2 topic pub --once /niihan/battery/state sensor_msgs/msg/BatteryState \
  "{percentage: 0.67, present: true, voltage: 24.2, power_supply_status: 2}"
```
This marks dashboard-received battery events as simulated. Do not use injected values as hardware evidence. Set `battery_simulation false` before using a real publisher. The parameter is read live on each battery message.

## 4. Alerts are robot-operational, not AI or app alerts

The existing JSON `telemetry` message now includes:
```json
{
  "home": {"configured": true, "source": "captured_live_tf", "pose": {"x": 0, "y": 0, "z": -0.54, "yaw": null}},
  "alerts": {
    "active": [{"code": "NAVIGATION_FAILED", "severity": "error", "message": "Nav2/mission navigation ended in failure.", "raised_at_utc": "2026-11-14T00:00:00+00:00"}],
    "events": [{"id": 1, "code": "NAVIGATION_FAILED", "transition": "raised", "severity": "error", "timestamp_utc": "2026-11-14T00:00:00+00:00", "timestamp_source": "system_utc"}]
  }
}
```
Event history is **in-memory, bounded to 60 transitions**, not a permanent audit log. `timestamp_utc` is system wall time, not ROS simulation time; do not use it for sensor synchronization. Transition events are emitted only when an alert changes state (no event flood at 10 Hz). The dashboard still emits telemetry every ~100ms; the client app must compare event IDs within the same session. IDs reset at restart. Alert JSON is additive; existing consumers can ignore unknown fields.

| Code | Condition | Severity |
|---|---|---|
| `NAVIGATION_FAILED` | Navigation status becomes FAILED | error |
| `LOCALIZATION_LOST` | Previously valid map-frame TF/pose becomes unavailable | critical |
| `ESTOP_ACTIVE` | Software stop asserted | critical |
| `GEOFENCE_BREACH` | Active GO geofence and valid pose outside footprint margin | critical |
| `DRIVE_UNHEALTHY` | Health supervisor reports drive fault | critical |
| `SENSOR_HEALTH_FAULT` | Missing/stale supervisor sensor reports incl. absent health heartbeat | error |
| `MANUAL_COMMAND_TIMEOUT` | Manual command stream stops after use; separate watchdog already stops drive | warning |
| `CAMERA_FEED_STALE` | Previously active front RGB feed absent for >3s | warning |
| `BATTERY_LOW` | Fresh, available battery <=20% | warning |
| `BATTERY_CRITICAL` | Fresh, available battery <=10% | critical |
| `BATTERY_TELEMETRY_STALE` | Previous battery message older than 5s | warning |
| `MOTION_STALLED` | >=2.5s commanded motion while fresh wheel feedback reports no motion | warning |
| `UNCOMMANDED_MOTION` | >=2.5s substantial wheel motion despite zero drive command | critical |

Motion anomalies are **diagnostic warnings only**, NOT automatic new movement commands. They may be affected by simulation timing, braking inertia, odometry validity and downstream controller behavior. Validate thresholds before using them as operational guarantees. An alert being absent does not certify safety.

This does NOT add fire, smoke, person detection, intruder recognition, AI hazard classification or external notifications; those remain AI/ML / application team responsibilities.

## 5. Site boundary guard

Existing `set_geofence` supplies **one inclusion GO polygon** and Nav2 costmap keepout masks. Example (illustrative; NOT a calibrated seaport boundary):
```json
{"action":"set_geofence","polygon":[{"x":-10,"y":-10},{"x":10,"y":-10},{"x":10,"y":10},{"x":-10,"y":10}]}
```
Run these **after** the polygon has been calibrated and validated:
```bash
ros2 param set /niihan_dashboard_node require_site_geofence true
ros2 param get /niihan_dashboard_node require_site_geofence
```
This parameter gates **dashboard-sourced motion requests**. Existing Nav2 keepout and ROS motion gateway should also be tested. It is NOT an upstream network firewall, a hardware E-stop, a proof that the current occupancy map includes the full site, or a guarantee against raw ROS topic injection. Multi-NO-GO polygons remain an additional client request, not implemented on this core branch.

## 6. Source-level checks before the Gazebo acceptance phase

```bash
cd ~/surveillance_project/niihan_ws
source /opt/ros/jazzy/setup.bash
# With the feature files applied, BEFORE starting a mission:
python3 -m py_compile src/niihan_dashboard/niihan_dashboard/operational_status.py \
  src/niihan_dashboard/niihan_dashboard/ros_bridge.py
colcon build --symlink-install --packages-select niihan_dashboard --parallel-workers 1
source install/setup.bash
python3 -m pytest -q src/niihan_dashboard/test/test_operational_status.py \
  src/niihan_dashboard/test/test_navigation_results.py
```
No claims about these test results are made in advance. Full T00/T01/T02/T05/T08/T11 Gazebo acceptance and physical integration occur separately in the subsequent verification phase. Preserve original client world/GLB, SDF resources, and known-good `./run.sh` launch.

## 7. MOU ownership boundary
- **Mayuresh/Prajwal:** Nav2, SLAM, mission control, robot health/events, safety gateway/geofence, camera/API data contract and source-level test coverage.
- **Sidd / client hardware:** Battery BMS publisher, voltage/charge source, motor/controller driver, physical docking/charging, E-stop power cut, wheel/sensor and stop-distance tests.
- **AI/ML engineer:** Fire/smoke/person/anomaly perception and model outputs.
- **App/frontend team:** Authenticated proxy, operator UI, alert display/acknowledgment, offline persistence, external notifications.

**Remaining release blocker:** The latest `site_robotics_seaport_collision_preview.sdf` and corresponding GLB/meshes must be packaged with approved rights before the client can reproduce the November demonstration from a fresh clone.
