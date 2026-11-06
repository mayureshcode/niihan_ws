"""Seaport Nav2 wrapper: select validated BT XML and launch the core stack.

Native Jazzy nav2_bringup/navigation_launch.py also manages routing and
docking, which this patrol application does not configure. Only the required
Nav2 nodes are managed by nav2_seaport_core.launch.py; collision monitoring
remains mandatory and commands retain NIIHAN motion gateway safety controls.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def _navigation_actions(context):
    description = get_package_share_directory('niihan_description')
    profile = LaunchConfiguration('bt_profile').perform(context).strip().lower()
    if profile not in ('stock', 'custom'):
        raise ValueError('bt_profile must be "stock" or "custom"')
    stack = LaunchConfiguration('nav2_stack_profile').perform(context).strip().lower()
    if stack != 'core':
        raise ValueError('Only nav2_stack_profile=core is supported for the seaport')

    if profile == 'stock':
        bt_dir = os.path.join(
            get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
        navigate_to_pose = os.path.join(
            bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml')
        navigate_through_poses = os.path.join(
            bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml')
    else:
        navigate_to_pose = LaunchConfiguration('bt_xml').perform(context)
        navigate_through_poses = os.path.join(
            description, 'config',
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
    core_launch = os.path.join(description, 'launch', 'nav2_seaport_core.launch.py')
    if not os.path.isfile(core_launch):
        raise FileNotFoundError('Seaport Nav2 core launch is missing: ' + core_launch)

    return [
        LogInfo(msg='Seaport Nav2 core lifecycle; BT profile: ' + profile +
                '; NavigateToPose: ' + navigate_to_pose),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
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
            get_package_share_directory('niihan_description'),
            'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=os.path.join(
            get_package_share_directory('niihan_description'),
            'config', 'navigate_to_pose.xml')),
        DeclareLaunchArgument('bt_profile', default_value='custom'),
        DeclareLaunchArgument('nav2_stack_profile', default_value='core'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=_navigation_actions),
    ])
