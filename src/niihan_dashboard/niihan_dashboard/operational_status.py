"""Hardware-neutral battery telemetry and transition-based robot operations alerts.

This module does not control motion or replace safety supervisors. Incoming battery
data is optional and is NEVER simulated unless an external test publisher labels it.
Time-based freshness uses a monotonic clock; event timestamps are UTC wall time.
"""
from collections import deque
from datetime import datetime, timezone
import math
import time


def _finite(value):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


class OperationalStatus:
    def __init__(self, clock=None, battery_timeout=5.0, history_limit=60):
        self.clock = clock or time.monotonic
        self.battery_timeout = float(battery_timeout)
        self._battery = None
        self._battery_at = None
        self._drive = None
        self._drive_at = None
        self._wheel = None
        self._wheel_at = None
        self._camera_at = None
        self._motion_kind = None
        self._motion_since = None
        self._ever_localized = False
        self._active = {}
        self._events = deque(maxlen=history_limit)
        self._event_id = 0

    def update_battery(self, percentage, status='unknown', voltage=None,
                       present=True, simulation=False):
        """BatteryState percentage is a FRACTION 0..1, not a human 0..100 value."""
        fraction = _finite(percentage)
        voltage = _finite(voltage)
        if not present or fraction is None or not 0 <= fraction <= 1:
            fraction = None
        charging = (True if status == 'charging' else
                    False if status in ('discharging', 'full', 'not_charging') else None)
        self._battery = {
            'percentage': round(fraction * 100, 1) if fraction is not None else None,
            'charging': charging if fraction is not None else None,
            'status': status if status in ('charging', 'discharging', 'full', 'not_charging') else 'unknown',
            'voltage': voltage if voltage is not None and voltage >= 0 else None,
            'source': '/niihan/battery/state',
            'simulation': bool(simulation),
        }
        self._battery_at = self.clock()

    def battery(self, now=None):
        now = self.clock() if now is None else now
        received = self._battery is not None
        age = max(0., now-self._battery_at) if received else None
        fresh = received and age <= self.battery_timeout
        report = {
            'available': bool(fresh and self._battery['percentage'] is not None),
            'fresh': bool(fresh),
            'percentage': self._battery['percentage'] if fresh else None,
            'charging': self._battery['charging'] if fresh else None,
            'status': self._battery['status'] if fresh else 'stale' if received else 'unavailable',
            'voltage': self._battery['voltage'] if fresh else None,
            'source': self._battery['source'] if received else None,
            'simulation': self._battery['simulation'] if received else None,
            'age_seconds': round(age, 2) if age is not None else None,
        }
        return report

    def update_drive(self, linear_x, angular_z):
        linear, angular = _finite(linear_x), _finite(angular_z)
        if linear is None or angular is None:
            return
        self._drive = (linear, angular)
        self._drive_at = self.clock()

    def update_wheel(self, linear_x, linear_y, angular_z):
        x, y, w = _finite(linear_x), _finite(linear_y), _finite(angular_z)
        if x is None or y is None or w is None:
            return
        self._wheel = (math.hypot(x, y), abs(w))
        self._wheel_at = self.clock()

    def camera_seen(self):
        self._camera_at = self.clock()

    def _motion_anomaly(self, now, enabled):
        if not enabled or self._drive is None or self._wheel is None or (
            now - self._drive_at > .6 or now - self._wheel_at > .6
        ):
            self._motion_kind = self._motion_since = None
            return None
        commanded = abs(self._drive[0]) >= .12 or abs(self._drive[1]) >= .20
        moving = self._wheel[0] >= .025 or self._wheel[1] >= .06
        uncommanded = (abs(self._drive[0]) <= .01 and abs(self._drive[1]) <= .02 and
                       (self._wheel[0] >= .12 or self._wheel[1] >= .20))
        kind = 'UNCOMMANDED_MOTION' if uncommanded else 'MOTION_STALLED' if commanded and not moving else None
        if kind != self._motion_kind:
            self._motion_kind, self._motion_since = kind, now
        if kind is not None and now - self._motion_since >= 2.5:
            return kind
        return None

    def evaluate(self, *, pose_valid, navigation_state, health=None, estop=False,
                 geofence_enabled=False, inside_geofence=True,
                 manual_command_timeout=False, now=None):
        """Evaluate only robot-operational signals; no camera/AI/behavior inference."""
        now = self.clock() if now is None else now
        self._ever_localized |= bool(pose_valid)
        health = health if isinstance(health, dict) else {}
        faults = health.get('faults', [])
        faults = faults if isinstance(faults, list) else []
        battery = self.battery(now)
        desired = {}

        def issue(code, severity, message):
            desired[code] = {'code': code, 'severity': severity, 'message': message}

        if manual_command_timeout:
            issue('MANUAL_COMMAND_TIMEOUT', 'warning', 'Manual command stream timed out; drive watchdog is active.')
        if self._camera_at is not None and now - self._camera_at > 3.0:
            issue('CAMERA_FEED_STALE', 'warning', 'Previously active front camera feed has stopped.')
        if estop:
            issue('ESTOP_ACTIVE', 'critical', 'Robot software emergency stop is active.')
        if self._ever_localized and not pose_valid:
            issue('LOCALIZATION_LOST', 'critical', 'Previously valid map-frame localization is unavailable.')
        if geofence_enabled and pose_valid and not inside_geofence:
            issue('GEOFENCE_BREACH', 'critical', 'Robot footprint is outside the permitted site boundary.')
        if navigation_state == 'FAILED':
            issue('NAVIGATION_FAILED', 'error', 'Nav2/mission navigation ended in failure.')
        if health.get('ready') is False:
            if 'drive_fault' in faults or 'drive' in faults:
                issue('DRIVE_UNHEALTHY', 'critical', 'Drive status is unhealthy or stale.')
            sensor_faults = [str(x) for x in faults if x not in ('drive', 'drive_fault')]
            if sensor_faults and 'waiting_for_supervisor' not in sensor_faults:
                issue('SENSOR_HEALTH_FAULT', 'error', 'Health supervisor reports: '+', '.join(sensor_faults[:6]))
        if self._battery is not None:
            if not battery['fresh']:
                issue('BATTERY_TELEMETRY_STALE', 'warning', 'Previously available battery telemetry is stale.')
            elif battery['available']:
                if battery['percentage'] <= 10.:
                    issue('BATTERY_CRITICAL', 'critical', 'Battery at or below 10%.')
                elif battery['percentage'] <= 20.:
                    issue('BATTERY_LOW', 'warning', 'Battery at or below 20%.')
        anomaly = self._motion_anomaly(now, enabled=bool(pose_valid) and not estop and health.get('ready') is True)
        if anomaly == 'MOTION_STALLED':
            issue(anomaly, 'warning', 'Drive commands sustained but wheel odometry indicates no motion.')
        elif anomaly == 'UNCOMMANDED_MOTION':
            issue(anomaly, 'critical', 'Wheel motion sustained while commanded velocity is zero.')

        for code in sorted(set(self._active) - set(desired)):
            self._event_id += 1
            self._events.append({
                'id': self._event_id, 'code': code, 'transition': 'cleared',
                'severity': self._active[code]['severity'],
                'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                'timestamp_source': 'system_utc',
            })
        for code in sorted(desired):
            if code in self._active:
                desired[code]['raised_at_utc'] = self._active[code]['raised_at_utc']
                continue
            self._event_id += 1
            raised = datetime.now(timezone.utc).isoformat()
            desired[code]['raised_at_utc'] = raised
            self._events.append({
                'id': self._event_id, 'code': code, 'transition': 'raised',
                'severity': desired[code]['severity'], 'message': desired[code]['message'],
                'timestamp_utc': raised, 'timestamp_source': 'system_utc',
            })
        self._active = desired

    def alerts(self):
        """Latest alert state and bounded transition history, never an unbounded log."""
        return {'active': list(self._active.values()), 'events': list(self._events)}
