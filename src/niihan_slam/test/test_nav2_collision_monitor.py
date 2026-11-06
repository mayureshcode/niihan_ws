"""Regression guard for the first observed Jazzy Nav2 lifecycle failure.

The real ROS startup previously aborted on:
collision_monitor: parameter 'observation_sources' is not initialized.
Tests validate the required scan source and that no Nav2 command bypasses
the collision monitor on its way to the command arbiter.
"""
import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
PARAMS = ROOT / 'src' / 'niihan_slam' / 'config' / 'nav2_3d_jazzy.yaml'
NAV_LAUNCH = ROOT / 'src' / 'niihan_description' / 'launch'


def test_collision_monitor_has_real_scan_source_and_stop_zone():
    config = yaml.safe_load(PARAMS.read_text())
    p = config['collision_monitor']['ros__parameters']
    assert p['enabled'] is True
    assert p['use_sim_time'] is True
    assert p['base_frame_id'] == 'base_nav'
    assert p['observation_sources'] == ['scan']
    assert p['scan']['topic'] == '/scan'
    assert p['scan']['type'] == 'scan'
    assert p['scan']['enabled'] is True
    assert p['polygons'] == ['RobotStop']
    assert p['RobotStop']['action_type'] == 'stop'
    assert p['RobotStop']['radius'] >= 0.45
    assert p['enable_stamped_cmd_vel'] is False
    assert p['cmd_vel_in_topic'] == '/cmd_vel_smoothed'
    assert p['cmd_vel_out_topic'] == '/niihan/cmd_vel/nav'


def test_seaport_nav2_launch_enforces_filtered_command_path():
    source = (NAV_LAUNCH / 'nav2_seaport_core.launch.py').read_text()
    ast.parse(source)
    assert "'collision_monitor'" in source
    assert "'nav2_collision_monitor'" in source
    assert "('cmd_vel', 'cmd_vel_nav')" in source
    assert "('cmd_vel', 'cmd_vel_smoothed')" in source
    assert "'route_server'" not in source.split("managed = [", 1)[1].split("]", 1)[0]
    assert "'docking_server'" not in source.split("managed = [", 1)[1].split("]", 1)[0]
    assert "'collision_monitor',\n        'bt_navigator'" in source


def test_navigation_wrapper_uses_seaport_core_not_upstream_launcher():
    source = (NAV_LAUNCH / 'niihan_navigation.launch.py').read_text()
    ast.parse(source)
    assert "'nav2_seaport_core.launch.py'" in source
    assert "'navigation_launch.py'" not in source
    assert "velocity_smoother:cmd_vel_smoothed" not in source
    assert "behavior_server:cmd_vel" not in source
