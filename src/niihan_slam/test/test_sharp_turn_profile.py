"""Static regression gates for the seaport sharp-turn controller candidate.

These tests are configuration checks, NOT proof of safe Gazebo operation.
Run with PYTHONPATH=src/niihan_slam and python3 -m pytest.
"""
from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'nav2_3d_jazzy.yaml'


def params():
    return yaml.safe_load(CONFIG.read_text())


def test_kinematic_shim_and_pose_progress_checker():
    controller = params()['controller_server']['ros__parameters']
    rpp = controller['FollowPath']
    progress = controller['progress_checker']
    assert rpp['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    assert rpp['primary_controller'] == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert rpp['rotate_to_heading_once'] is True
    assert rpp['max_cost_threshold'] <= 253.0
    assert rpp['simulate_ahead_time'] >= 1.0
    assert rpp['allow_reversing'] is False
    assert rpp['use_collision_detection'] is True
    assert rpp['max_allowed_time_to_collision_up_to_carrot'] >= .5
    assert progress['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert 0.2 <= progress['required_movement_angle'] <= .6
    assert progress['required_movement_radius'] > 0


def test_turn_speed_is_consistent_with_safety_gateway():
    controller = params()['controller_server']['ros__parameters']['FollowPath']
    assert 0.1 < controller['regulated_linear_scaling_min_speed'] <= controller['desired_linear_vel']
    assert 0 < controller['lookahead_dist'] <= .5
    assert controller['min_lookahead_dist'] <= controller['lookahead_dist']
    assert controller['lookahead_dist'] <= controller['max_lookahead_dist']
    assert controller['rotate_to_heading_angular_vel'] <= .6


def test_geofence_unknown_space_and_stop_zone_are_preserved():
    values = params()
    planner = values['planner_server']['ros__parameters']['GridBased']
    assert planner['allow_unknown'] is False
    for name in ('global_costmap', 'local_costmap'):
        layer = values[name][name]['ros__parameters']
        assert layer['keepout_filter']['enabled'] is True
        assert layer['cliff_layer']['enabled'] is False  # no verified cliff scan
        assert layer['inflation_layer']['inflation_radius'] >= .8
    monitor = values['collision_monitor']['ros__parameters']
    assert 'StopZone' in monitor['polygons']
    assert monitor['StopZone']['enabled'] is True
    assert monitor['scan']['enabled'] is True


def test_robot_footprint_unchanged_by_controller_tuning():
    values = params()
    expected = '[[-0.35, -0.25], [-0.35, 0.25], [0.35, 0.25], [0.35, -0.25]]'
    assert values['global_costmap']['global_costmap']['ros__parameters']['footprint'] == expected
    assert values['local_costmap']['local_costmap']['ros__parameters']['footprint'] == expected
