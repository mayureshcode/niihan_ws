"""Static regression checks for reversible Jazzy Nav2 behavior-tree selection.

These verify source/configuration only. Live Nav2 lifecycle acceptance requires
the target Ubuntu 24.04/Jazzy simulator.
"""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET


SLAM_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = SLAM_ROOT.parent


def test_jazzy_simulation_exposes_stock_and_custom_profiles():
    path = SLAM_ROOT / 'launch' / 'slam_simulation.launch.py'
    source = path.read_text()
    ast.parse(source, filename=str(path))
    assert "'nav2_bt_profile','stock' if os.environ.get('ROS_DISTRO')=='jazzy'" in source
    assert "'bt_profile':LaunchConfiguration('nav2_bt_profile')" in source
    assert "'spawn_z':LaunchConfiguration('spawn_z')" in source


def test_navigation_launch_selects_both_stock_tree_paths():
    path = SOURCE_ROOT / 'niihan_description' / 'launch' / 'niihan_navigation.launch.py'
    source = path.read_text()
    ast.parse(source, filename=str(path))
    assert "'navigate_to_pose_w_replanning_and_recovery.xml'" in source
    assert "'navigate_through_poses_w_replanning_and_recovery.xml'" in source
    assert "('stock', 'custom')" in source
    assert "'default_nav_to_pose_bt_xml': navigate_to_pose" in source
    assert "'default_nav_through_poses_bt_xml': navigate_through_poses" in source


def test_preserve_both_original_custom_trees():
    pose = SLAM_ROOT / 'config' / 'navigate_to_pose_position_jazzy.xml'
    through = SOURCE_ROOT / 'niihan_description' / 'config' / 'navigate_through_poses_jazzy.xml'
    for path in (pose, through):
        tree = ET.parse(path)
        assert tree.getroot().tag == 'root'
        assert tree.find('.//BehaviorTree') is not None
