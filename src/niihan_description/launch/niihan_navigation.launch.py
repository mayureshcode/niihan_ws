"""Native Jazzy Nav2 core bringup, with an opt-in upstream fallback.

The upstream Nav2 Jazzy bringup includes docking/route servers that this
seaport patrol does not use. Its lifecycle manager may abort on their missing
configuration. The core profile retains collision_monitor, configured with
/scan and a stop zone, in the mandatory lifecycle sequence.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


# Core navigation only; routing/docking are deliberately not enabled until
# their real project-specific parameters exist. Keep collision_monitor active.
CORE_NODES = (
    ('nav2_controller', 'controller_server'),
    ('nav2_smoother', 'smoother_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
    ('nav2_collision_monitor', 'collision_monitor'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower'),
)


def _navigation_actions(context):
    description = get_package_share_directory('niihan_description')
    profile = LaunchConfiguration('bt_profile').perform(context).strip().lower()
    if profile not in ('stock', 'custom'):
        raise ValueError('bt_profile must be "stock" or "custom"')
    stack = LaunchConfiguration('nav2_stack_profile').perform(context).strip().lower()
    if stack not in ('core', 'upstream'):
        raise ValueError('nav2_stack_profile must be "core" or "upstream"')

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

    # Mirror the installed nav2_bringup parameter mechanism.
    params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key='',
            param_rewrites={
                'default_nav_to_pose_bt_xml': navigate_to_pose,
                'default_nav_through_poses_bt_xml': navigate_through_poses,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    info = LogInfo(msg='Nav2 lifecycle profile: ' + stack +
                   '; BT profile: ' + profile +
                   '; NavigateToPose: ' + navigate_to_pose)

    if stack == 'upstream':
        # Legacy behavior for comparison only: upstream launches route and
        # docking servers as well, requiring site-specific parameters.
        return [info, GroupAction(actions=[
            SetRemap(src='velocity_smoother:cmd_vel_smoothed',
                     dst='/niihan/cmd_vel/nav'),
            SetRemap(src='behavior_server:cmd_vel',
                     dst='/niihan/cmd_vel/nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': params,
                    'autostart': LaunchConfiguration('autostart'),
                    'use_composition': 'False',
                }.items(),
            ),
        ])]

    # This is the standard Nav2 Jazzy velocity chain:
    # controller/behavior -> /cmd_vel_nav -> velocity_smoother ->
    # /cmd_vel_smoothed -> collision_monitor -> /niihan/cmd_vel/nav ->
    # NIIHAN command arbiter/motion gateway. No direct route around monitor.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nodes = []
    for package, executable in CORE_NODES:
        node_remappings = list(remappings)
        if executable in ('controller_server', 'behavior_server', 'velocity_smoother'):
            node_remappings.append(('cmd_vel', 'cmd_vel_nav'))
        nodes.append(Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=[params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=node_remappings,
        ))
    lifecycle_names = [name for _, name in CORE_NODES]
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': lifecycle_names,
        }],
    ))
    return [info] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('niihan_description'), 'config',
            'nav2_params.yaml')),
        DeclareLaunchArgument('bt_xml', default_value=os.path.join(
            get_package_share_directory('niihan_description'), 'config',
            'navigate_to_pose.xml')),
        DeclareLaunchArgument('bt_profile', default_value='custom'),
        DeclareLaunchArgument('nav2_stack_profile', default_value='core',
                              description='core: seaport patrol with collision '
                              'monitor; upstream: full Nav2 demo stack'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=_navigation_actions),
    ])
