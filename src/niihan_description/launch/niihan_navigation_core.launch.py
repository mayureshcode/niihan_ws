"""NIIHAN's non-composed Nav2 patrol stack (no charging-dock server).

This is intentionally project-owned: do not modify vendored navigation2.
All navigation Twist commands go through velocity_smoother, then
collision_monitor, before reaching /niihan/cmd_vel/nav.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


# Docking is not part of the surveillance rover's patrol capability.
MANAGED_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key='',
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    def nav_node(package, executable, name, remappings=None):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[params],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
            + (remappings or []),
        )

    # The controller and recovery behaviors feed the velocity smoother.
    # Collision Monitor alone publishes the final navigation command.
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('autostart', default_value='true'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', use_sim_time),
        nav_node('nav2_controller', 'controller_server', 'controller_server',
                 [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', 'behavior_server',
                 [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                 [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor',
                 [('cmd_vel', '/niihan/cmd_vel/nav')]),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': MANAGED_NODES},
            ],
        ),
    ])
