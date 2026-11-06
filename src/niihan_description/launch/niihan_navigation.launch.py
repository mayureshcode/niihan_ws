"""Native Nav2 bringup with selectable stock or project behavior trees.

Use stock Nav2 trees to isolate Jazzy lifecycle failures before testing the
project's position-arrival / replanning tree. This never bypasses safety gates.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    LogInfo, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def _navigation_actions(context):
    package_dir = get_package_share_directory('niihan_description')
    profile = LaunchConfiguration('bt_profile').perform(context).strip().lower()
    if profile not in ('stock', 'custom'):
        raise ValueError('bt_profile must be "stock" or "custom"; got ' + repr(profile))

    if profile == 'stock':
        # Absolute package paths, rather than empty parameters, avoid ambiguity
        # over whether a predeclared empty default selects a stock tree.
        bt_dir = os.path.join(
            get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
        navigate_to_pose = os.path.join(
            bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml')
        navigate_through_poses = os.path.join(
            bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml')
    else:
        navigate_to_pose = LaunchConfiguration('bt_xml').perform(context)
        navigate_through_poses = os.path.join(
            package_dir, 'config',
            'navigate_through_poses_jazzy.xml' if os.environ.get('ROS_DISTRO') == 'jazzy'
            else 'navigate_through_poses.xml')

    for name, path in [('NavigateToPose BT', navigate_to_pose),
                       ('NavigateThroughPoses BT', navigate_through_poses)]:
        if not os.path.isfile(path):
            raise FileNotFoundError(name + ' file is missing: ' + path)

    params_file = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': navigate_to_pose,
            'default_nav_through_poses_bt_xml': navigate_through_poses,
        },
        convert_types=True,
    )
    return [
        LogInfo(msg='Nav2 BT profile: ' + profile +
                '; pose=' + navigate_to_pose),
        # Seaport-specific Nav2 bringup keeps collision_monitor in the
        # command path, without launching unrelated route/docking servers.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_dir, 'launch', 'nav2_seaport_core.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': params_file,
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('niihan_description'), 'config',
            'nav2_params.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=os.path.join(
            get_package_share_directory('niihan_description'), 'config',
            'navigate_to_pose.xml')),
        DeclareLaunchArgument('bt_profile', default_value='custom',
                              description='stock: standard Nav2 BT; '
                              'custom: project BT XML'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=_navigation_actions),
    ])
