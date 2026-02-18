#!/usr/bin/env python3
"""
butler_simulation.launch.py
============================
Complete fixed launch — v4 (butler_robot package).

ROOT CAUSE of crashes fixed here:
  Problem 1 → gzserver exit 255:
    The previous launch was passing nav2_params.yaml to gzserver via
    --params-file. That file has ROS2 node params, NOT Gazebo params.
    Gazebo can't parse it and immediately crashes (exit 255).
    FIX: Do NOT pass any --params-file to gzserver. Use plain world only.

  Problem 2 → "Entity [waffle] already exists":
    Old gzserver process was still running when new launch started.
    TimerAction(2s) is not enough because gzserver from previous run
    keeps the Gazebo world alive. FIX: Kill all gazebo processes before
    launching using ExecuteProcess + OnProcessExit event chain.

Map frame coordinates (verified within map bounds -6.24..9.91 x, -5.25..6.25 y):
  home    : (-0.812,  4.70)  ← AMCL initial pose
  kitchen : (-4.04,   3.99)
  table1  : (-0.7981, 1.44)
  table2  : ( 6.87,   1.54)
  table3  : ( 7.57,  -2.41)

Gazebo world frame spawn: (-2.854, 4.236)

Author: Arunesh
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────────
    pkg_turtlebot    = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_butler       = get_package_share_directory('butler_robot')

    # ── File paths ────────────────────────────────────────────────────────────
    world_file  = os.path.join(pkg_turtlebot, 'worlds', 'turtlebot3_house.world')
    map_file    = os.path.join(pkg_turtlebot, 'maps',   'map.yaml')
    nav2_params = os.path.join(pkg_turtlebot, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # ── Launch configurations ─────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart    = LaunchConfiguration('autostart',    default='true')
    map_yaml     = LaunchConfiguration('map',          default=map_file)
    params_file  = LaunchConfiguration('params_file',  default=nav2_params)

    # ─────────────────────────────────────────────────────────────────────────
    # PRE-LAUNCH: Kill any leftover Gazebo so we always start clean
    # This is the only reliable fix for "entity already exists"
    # ─────────────────────────────────────────────────────────────────────────
    kill_gazebo = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f gzserver 2>/dev/null || true; '
             'pkill -9 -f gzclient 2>/dev/null || true; '
             'pkill -9 -f gazebo   2>/dev/null || true; '
             'sleep 2; echo "[butler_launch] Gazebo cleanup done"'],
        output='screen',
        name='kill_old_gazebo'
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Gazebo server — starts AFTER kill completes
    # NOTE: Do NOT pass nav2_params.yaml here — that crashes gzserver!
    # ─────────────────────────────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # Chain: kill finishes → start gzserver
    gzserver_after_kill = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=kill_gazebo,
            on_exit=[gzserver]
        )
    )

    return LaunchDescription([

        # ── Launch arguments ──────────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('autostart',    default_value='true',
                              description='Auto-start nav2 lifecycle nodes'),
        DeclareLaunchArgument('map',          default_value=map_file,
                              description='Full path to map YAML'),
        DeclareLaunchArgument('params_file',  default_value=nav2_params,
                              description='Full path to Nav2 params YAML'),

        # ── 1. Kill stale Gazebo, then start fresh gzserver ───────────────────
        kill_gazebo,
        gzserver_after_kill,

        # ── 2. Gazebo client GUI (wait 4s for gzserver to be ready) ──────────
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
            ),
        ]),

        # ── 3. Robot State Publisher ──────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot, 'launch',
                             'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # ── 4. Spawn TurtleBot3 (wait 6s for gzserver to fully load world) ───
        # World frame (-2.854, 4.236) → map frame home (-0.812, 4.70)
        TimerAction(period=6.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_turtlebot, 'launch',
                                 'spawn_turtlebot3.launch.py')),
                launch_arguments={
                    'x_pose': '-2.854',
                    'y_pose': '4.236',
                    'z_pose': '0.01',
                    'yaw':    '0.0',
                }.items()
            ),
        ]),

        # ── 5. Map Server ─────────────────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml}
            ]
        ),

        # ── 6. AMCL ───────────────────────────────────────────────────────────
        # initial_pose = home position in MAP frame
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time':     use_sim_time},
                {'set_initial_pose': True},
                {'initial_pose.x':   -0.812},
                {'initial_pose.y':    4.70},
                {'initial_pose.z':    0.0},
                {'initial_pose.yaw':  0.0},
            ]
        ),

        # ── 7. Controller Server ──────────────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),

        # ── 8. Planner Server ─────────────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # ── 9. Behavior Server ────────────────────────────────────────────────
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),

        # ── 10. BT Navigator ──────────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # ── 11. Waypoint Follower ─────────────────────────────────────────────
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),

        # ── 12. Velocity Smoother ─────────────────────────────────────────────
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/cmd_vel',          '/cmd_vel_nav'),
                ('/cmd_vel_smoothed', '/cmd_vel')
            ]
        ),

        # ── 13. Lifecycle Manager – Localization ──────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart':    autostart},
                {'node_names':   ['map_server', 'amcl']}
            ]
        ),

        # ── 14. Lifecycle Manager – Navigation ────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart':    autostart},
                {'node_names': [
                    'controller_server', 'planner_server',
                    'behavior_server',   'bt_navigator',
                    'waypoint_follower', 'velocity_smoother'
                ]}
            ]
        ),

        # ── 15. Butler Node ───────────────────────────────────────────────────
        # NOTE: package changed to 'butler_robot'
        Node(
            package='butler_robot',
            executable='butler_node',
            name='butler_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # ── 16. RViz2 ─────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

    ])
