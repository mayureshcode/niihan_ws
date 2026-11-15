"""Route preflight guard regressions independent of Gazebo."""
from types import SimpleNamespace as NS

from niihan_dashboard.geofence_manager import GeofenceManager
from niihan_dashboard.route_policy import assess_path


def p(x, y):
    return NS(pose=NS(position=NS(x=x, y=y)))


def test_short_clear_path_is_allowed():
    result = assess_path([p(0, 0), p(1, 0), p(2, 0)], 2, 0)
    assert result['ok']
    assert result['length_m'] == 2.0


def test_empty_path_is_rejected():
    result = assess_path([], 2, 0)
    assert not result['ok']


def test_wrong_goal_endpoint_is_rejected():
    result = assess_path([p(0, 0), p(1, 0)], 5, 0)
    assert not result['ok'] and 'endpoint' in result['message']


def test_forty_meter_detour_for_four_meter_goal_is_rejected():
    result = assess_path(
        [p(0, 0), p(20, 0), p(20, 4), p(0, 4)], 0, 4)
    assert not result['ok']
    assert 'Excessive detour' in result['message']


def test_legit_detour_below_limit_is_accepted():
    result = assess_path(
        [p(0, 0), p(1, 1), p(2, 0), p(3, 0)], 3, 0)
    assert result['ok']


def test_route_exiting_go_zone_is_rejected_even_when_goal_is_inside():
    gf = GeofenceManager()
    assert gf.set_geofence([
        {'x': 0, 'y': 0}, {'x': 5, 'y': 0},
        {'x': 5, 'y': 5}, {'x': 0, 'y': 5}
    ])[0]
    result = assess_path([p(2, 2), p(6, 2), p(3, 3)], 3, 3, gf, .45)
    assert not result['ok']
    assert 'geofence' in result['message']


def test_route_safely_inside_go_zone_is_accepted():
    gf = GeofenceManager()
    assert gf.set_geofence([
        {'x': 0, 'y': 0}, {'x': 5, 'y': 0},
        {'x': 5, 'y': 5}, {'x': 0, 'y': 5}
    ])[0]
    result = assess_path([p(1, 1), p(2, 2), p(3, 3)], 3, 3, gf, .45)
    assert result['ok']


def test_plan_start_must_match_localized_robot():
    result = assess_path([p(8, 8), p(9, 8)], 9, 8, start_x=0, start_y=0)
    assert not result['ok']
    assert 'starts' in result['message']


def test_single_point_path_is_not_sufficient():
    result = assess_path([p(2, 0)], 2, 0)
    assert not result['ok']
