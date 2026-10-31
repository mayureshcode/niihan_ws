"""ROS-independent contract checks for the external-stack seaport launch.

These checks intentionally prevent the silent regression in which Gazebo's
LiDAR cloud arrives on the legacy remapped topic but SensorAdapter subscribes
to its unrelated default raw topic, leaving /scan and health missing.
They are static wiring tests, not live sensor acceptance tests.
"""
import ast
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
LAUNCH = SRC / 'niihan_slam' / 'launch' / 'slam_simulation.launch.py'
GAZEBO = SRC / 'niihan_description' / 'launch' / 'niihan_gazebo.launch.py'
ADAPTER = SRC / 'niihan_bringup' / 'niihan_bringup' / 'common_nodes.py'


def _text(path):
    source = path.read_text(encoding='utf-8')
    ast.parse(source, filename=str(path))
    return source


def test_seaport_external_lidar_route_matches_adapter_input():
    gazebo = _text(GAZEBO)
    launch = _text(LAUNCH)
    adapter = _text(ADAPTER)
    assert "'external_stack':'true'" in launch
    assert "'/niihan/legacy/lidar/points'" in gazebo
    assert "('/niihan/sensors/lidar/points', PythonExpression" in gazebo
    assert "'cloud_topic':'/niihan/legacy/lidar/points'" in launch
    assert "self.create_subscription(PointCloud2, cloud, self.cloud, qos_profile_sensor_data)" in adapter
    assert "self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)" in adapter


def test_scan_has_no_ground_truth_or_permission_bypass():
    launch = _text(LAUNCH)
    adapter = _text(ADAPTER)
    assert "'gravity_aligned_height':True" in launch
    assert "'scan_frame':'base_nav'" in launch
    assert "'require_health':True" in launch
    assert "'drive_enabled','false'" in launch
    assert "self.allowed.publish(permission)" in adapter
    assert "self.adapter_status.publish" in adapter


def test_camera_comes_from_sim_bridge_not_duplicate_adapter():
    launch = _text(LAUNCH)
    gazebo = _text(GAZEBO)
    assert "'relay_camera':False" in launch
    assert "'/niihan/sensors/panoramic/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'" in gazebo
