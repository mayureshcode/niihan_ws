"""NIIHAN Nav2 parameters and project-owned patrol bringup.

The patrol stack intentionally omits charging-dock functionality, while
retaining planner, controller, behavior, waypoint and collision-monitor nodes.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_dir = get_package_share_directory('niihan_description')
    params_file = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': LaunchConfiguration('bt_xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                package_dir, 'config',
                'navigate_through_poses_jazzy.xml'
                if os.environ.get('ROS_DISTRO') == 'jazzy'
                else 'navigate_through_poses.xml'),
        },
        convert_types=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_dir, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'bt_xml',
            default_value=os.path.join(package_dir, 'config', 'navigate_to_pose.xml'),
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_dir, 'launch', 'niihan_navigation_core.launch.py',
            )),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': params_file,
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ])
