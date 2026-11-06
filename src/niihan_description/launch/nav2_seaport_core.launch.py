"""Nav2 Jazzy seaport core with a real collision-monitor velocity chain.

Uses only the Nav2 servers required for map-based waypoint navigation.
The upstream Jazzy navigation_launch.py starts additional route and docking
servers that are not configured or used by the NIIHAN seaport mission.
Do not bypass collision_monitor: it is the sole ROS navigation output into
the project's command arbiter and health-gated motion gateway.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    parameters = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key='',
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    # Deliberately exclude route_server and docking_server: neither is used by
    # the existing waypoint mission manager, and docking has no dock database.
    managed = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def nav_node(package, executable, name, extra_remaps=()):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[parameters],
            remappings=tf + list(extra_remaps),
        )

    # Controller -> velocity smoother -> collision monitor -> command arbiter
    # -> motion gateway -> Gazebo drive.
    #
    # Behavior-server recovery commands use the SAME collision monitor input
    # instead of publishing directly to the arbiter.
    actions = [
        SetParameter(name='use_sim_time', value=use_sim_time),
        nav_node('nav2_controller', 'controller_server', 'controller_server',
                 [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', 'behavior_server',
                 [('cmd_vel', 'cmd_vel_smoothed')]),
        nav_node('nav2_velocity_smoother', 'velocity_smoother',
                 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        nav_node('nav2_collision_monitor', 'collision_monitor',
                 'collision_monitor'),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav_node('nav2_waypoint_follower', 'waypoint_follower',
                 'waypoint_follower'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'autostart': autostart},
                {'node_names': managed},
                {'use_sim_time': use_sim_time},
            ],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        GroupAction(actions=actions),
    ])
