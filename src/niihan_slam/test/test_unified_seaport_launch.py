"""Source-only safety and integration contract for the one-command seaport launch.

Passing these tests is not a substitute for Jazzy/Gazebo runtime validation.
"""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SLAM = ROOT / "src" / "niihan_slam"
DASH = ROOT / "src" / "niihan_dashboard"


def test_delayed_nav2_startup_is_reversible():
    launch = (SLAM / "launch" / "slam_simulation.launch.py").read_text()
    ast.parse(launch)
    assert "TimerAction(period=float(LaunchConfiguration('nav2_start_delay')" in launch
    assert "'nav2_start_delay','15.0' if os.environ.get('ROS_DISTRO')=='jazzy'" in launch
    assert "'bt_profile':LaunchConfiguration('nav2_bt_profile')" in launch


def test_camera_frames_not_rotated_twice():
    bridge = (DASH / "niihan_dashboard" / "ros_bridge.py").read_text()
    ast.parse(bridge)
    assert "self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')" in bridge
    assert "cv2.ROTATE_180" not in bridge


def test_spawn_height_forwarded_to_gazebo():
    launch = (SLAM / "launch" / "slam_simulation.launch.py").read_text()
    gazebo = (ROOT / "src" / "niihan_description" / "launch" /
              "niihan_gazebo.launch.py").read_text()
    assert "'spawn_z':LaunchConfiguration('spawn_z')" in launch
    assert "'-z', LaunchConfiguration('spawn_z')" in gazebo
    assert "'gravity_aligned_height':False" in launch


def test_integrated_launcher_preserves_real_safety_and_pinned_glim():
    runner = (ROOT / "scripts" / "start_seaport.sh").read_text()
    assert '"$glim/install/glim_ros"' in runner
    assert '"$glim/scripts/environment.sh"' in runner
    assert 'ldd -r "$binary"' in runner
    assert "niihan_description niihan_slam niihan_dashboard" in runner
    assert "nav2_bt_profile:=stock nav2_start_delay:=15.0" in runner
    assert "scripts/nav2_lifecycle_report.py" in runner
    assert "ros2 launch niihan_slam slam_simulation.launch.py" in runner
    assert "ground_truth" not in runner
