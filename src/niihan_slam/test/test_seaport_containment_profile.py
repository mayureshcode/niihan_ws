"""Static guardrails for the Jazzy seaport containment profile.

These tests verify configuration only. They do NOT prove the physical/Gazebo
edge is fenced, map origin is calibrated, or live collision safety.
"""
from pathlib import Path
import ast
import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_seaport_navigation_rejects_unknown_space():
    config = yaml.safe_load((ROOT / 'config/nav2_3d_jazzy.yaml').read_text())
    params = config['planner_server']['ros__parameters']['GridBased']
    assert params['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert params['allow_unknown'] is False


def test_cliff_layer_is_not_misrepresented_as_active():
    config = yaml.safe_load((ROOT / 'config/nav2_3d_jazzy.yaml').read_text())
    for scope in ('global_costmap', 'local_costmap'):
        params = config[scope][scope]['ros__parameters']
        assert params['cliff_layer']['enabled'] is False
        assert params['keepout_filter']['enabled'] is True
        assert params['keepout_filter']['filter_info_topic'] == '/niihan/geofence_filter_info'


def test_seaport_launch_fail_closed_parameter_is_passed_to_dashboard():
    source = (ROOT / 'launch/slam_simulation.launch.py').read_text()
    ast.parse(source)
    assert "site_geofence_required = 'seaport' in world.lower() or 'seaport' in environment.lower()" in source
    assert "'require_site_geofence':site_geofence_required" in source


def test_collision_monitor_remains_enabled_in_seaport_config():
    config = yaml.safe_load((ROOT / 'config/nav2_3d_jazzy.yaml').read_text())
    monitor = config['collision_monitor']['ros__parameters']
    assert monitor['StopZone']['enabled'] is True
    assert monitor['scan']['enabled'] is True
    assert 'StopZone' in monitor['polygons']
