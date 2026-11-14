"""Pure-Python regression tests: battery/alerts do not need Gazebo or ROS graph."""
from niihan_dashboard.operational_status import OperationalStatus


def test_missing_battery_is_unknown_not_zero():
    service = OperationalStatus()
    battery = service.battery()
    assert not battery['available'] and not battery['fresh']
    assert battery['percentage'] is None and battery['charging'] is None
    service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    assert not any(a['code'].startswith('BATTERY_') for a in service.alerts()['active'])


def test_battery_uses_fraction_percent_and_reports_charging():
    now = [1.]
    service = OperationalStatus(clock=lambda: now[0])
    service.update_battery(.67, 'charging', voltage=24.2, simulation=True)
    battery = service.battery()
    assert battery['percentage'] == 67.
    assert battery['charging'] is True
    assert battery['simulation'] is True
    assert battery['voltage'] == 24.2


def test_stale_battery_never_reuses_percentage_as_fresh():
    now = [0.]
    service = OperationalStatus(clock=lambda: now[0])
    service.update_battery(.30, 'discharging')
    now[0] = 6.
    battery = service.battery()
    assert not battery['fresh'] and not battery['available']
    assert battery['percentage'] is None and battery['charging'] is None
    assert battery['status'] == 'stale'
    service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    assert 'BATTERY_TELEMETRY_STALE' in {x['code'] for x in service.alerts()['active']}


def test_invalid_battery_never_becomes_valid_percentage():
    service = OperationalStatus()
    for value in (float('nan'), float('inf'), -.1, 1.1, None):
        service.update_battery(value, 'charging')
        assert service.battery()['percentage'] is None
        assert service.battery()['charging'] is None


def test_low_and_critical_battery_transitions_deduplicated():
    now = [1.]
    service = OperationalStatus(clock=lambda: now[0])
    service.update_battery(.19, 'discharging')
    for _ in range(30):
        service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    events = service.alerts()['events']
    assert len(events) == 1 and events[0]['code'] == 'BATTERY_LOW'
    service.update_battery(.08, 'discharging')
    service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    assert {x['code'] for x in service.alerts()['active']} == {'BATTERY_CRITICAL'}
    assert {(x['code'], x['transition']) for x in service.alerts()['events']} == {
        ('BATTERY_LOW', 'raised'), ('BATTERY_LOW', 'cleared'), ('BATTERY_CRITICAL', 'raised')}


def test_navigation_fault_and_estop_transition_then_clear():
    service = OperationalStatus()
    service.evaluate(pose_valid=True, navigation_state='FAILED', health={'ready': True}, estop=True)
    assert {'NAVIGATION_FAILED', 'ESTOP_ACTIVE'} == {x['code'] for x in service.alerts()['active']}
    service.evaluate(pose_valid=True, navigation_state='RUNNING', health={'ready': True}, estop=False)
    assert not service.alerts()['active']
    assert sum(x['transition'] == 'cleared' for x in service.alerts()['events']) == 2


def test_health_faults_and_localization_loss_are_separate():
    service = OperationalStatus()
    service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    service.evaluate(pose_valid=False, navigation_state='RUNNING',
                     health={'ready': False, 'faults': ['drive_fault', '/scan']})
    assert {'LOCALIZATION_LOST', 'DRIVE_UNHEALTHY', 'SENSOR_HEALTH_FAULT'} == {
        x['code'] for x in service.alerts()['active']}


def test_startup_without_first_pose_does_not_raise_localization_lost():
    service = OperationalStatus()
    service.evaluate(pose_valid=False, navigation_state='IDLE',
                     health={'ready': False, 'faults': ['waiting_for_supervisor']})
    assert service.alerts()['active'] == []


def test_manual_timeout_and_stale_camera_events():
    now = [1.]
    service = OperationalStatus(clock=lambda: now[0])
    service.camera_seen()
    now[0] = 5.
    service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True},
                     manual_command_timeout=True)
    assert {'CAMERA_FEED_STALE', 'MANUAL_COMMAND_TIMEOUT'} == {
        x['code'] for x in service.alerts()['active']}
    service.camera_seen()
    service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    assert service.alerts()['active'] == []


def test_drive_stall_needs_sustained_fresh_evidence():
    now = [0.]
    service = OperationalStatus(clock=lambda: now[0])
    for t in (0., 1., 2., 2.6):
        now[0] = t
        service.update_drive(.2, 0.)
        service.update_wheel(0., 0., 0.)
        service.evaluate(pose_valid=True, navigation_state='RUNNING', health={'ready': True})
    assert 'MOTION_STALLED' in {x['code'] for x in service.alerts()['active']}
    now[0] = 3.
    service.update_drive(0., 0.)
    service.update_wheel(0., 0., 0.)
    service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    assert 'MOTION_STALLED' not in {x['code'] for x in service.alerts()['active']}


def test_stale_feedback_cannot_generate_motion_mismatch():
    now = [0.]
    service = OperationalStatus(clock=lambda: now[0])
    service.update_drive(.3, 0.)
    service.update_wheel(0., 0., 0.)
    now[0] = 10.
    service.evaluate(pose_valid=True, navigation_state='RUNNING', health={'ready': True})
    assert 'MOTION_STALLED' not in {x['code'] for x in service.alerts()['active']}


def test_uncommanded_motion_requires_sustained_evidence():
    now = [0.]
    service = OperationalStatus(clock=lambda: now[0])
    for t in (0., 1., 2., 2.8):
        now[0] = t
        service.update_drive(0., 0.)
        service.update_wheel(.18, 0., 0.)
        service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    assert 'UNCOMMANDED_MOTION' in {x['code'] for x in service.alerts()['active']}


def test_transition_history_is_bounded():
    service = OperationalStatus(history_limit=8)
    for _ in range(30):
        service.evaluate(pose_valid=True, navigation_state='FAILED', health={'ready': True})
        service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True})
    assert len(service.alerts()['events']) == 8


def test_geofence_alert_only_for_known_breach():
    service = OperationalStatus()
    service.evaluate(pose_valid=True, navigation_state='IDLE', health={'ready': True},
                     geofence_enabled=True, inside_geofence=False)
    assert 'GEOFENCE_BREACH' in {x['code'] for x in service.alerts()['active']}
