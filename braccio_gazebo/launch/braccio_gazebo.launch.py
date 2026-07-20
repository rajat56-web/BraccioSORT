#!/usr/bin/env python3
"""
braccio_gazebo.launch.py — fixed spawn method
Key change: pre-resolve xacro → URDF file via OpaqueFunction, then
spawn from file (not topic). This preserves <gazebo> plugin blocks that
Gazebo Harmonic drops when converting from the robot_description topic.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


# Resolved URDF written here before Gazebo spawns
RESOLVED_URDF_PATH = '/tmp/braccio_resolved.urdf'


def resolve_xacro(context, *args, **kwargs):
    """OpaqueFunction: run xacro and write resolved URDF to /tmp."""
    desc_pkg = get_package_share_directory('braccio_description')
    xacro_path = os.path.join(desc_pkg, 'urdf', 'braccio.urdf.xacro')

    result = subprocess.run(
        ['xacro', xacro_path],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        raise RuntimeError(f'xacro failed:\n{result.stderr}')

    with open(RESOLVED_URDF_PATH, 'w') as f:
        f.write(result.stdout)

    print(f'[resolve_xacro] Written to {RESOLVED_URDF_PATH}')
    return []


def generate_launch_description():

    gz_pkg   = get_package_share_directory('braccio_gazebo')
    desc_pkg = get_package_share_directory('braccio_description')

    # ── Environment ───────────────────────────────────────────────────────
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(gz_pkg, '..'),
    )

    # ── Arguments ─────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gz_pkg, 'worlds', 'braccio_sorting.world'),
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )
    world        = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Step 1: resolve xacro before anything else runs ───────────────────
    xacro_resolver = OpaqueFunction(function=resolve_xacro)

    # ── Step 2: Robot State Publisher (uses resolved file) ─────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat', ' ', RESOLVED_URDF_PATH]),
                value_type=str,
            ),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ── Step 3: Gazebo ────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
    )

    # ── Step 4: Spawn from FILE (not topic) — preserves plugin blocks ─────
    spawn_robot = TimerAction(
        period=5.0,   # wait for Gazebo to fully start
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'braccio',
                    '-file', RESOLVED_URDF_PATH,
                    '-x', '0.0', '-y', '0.0', '-z', '0.05',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    # ── Step 5: Bridges ───────────────────────────────────────────────────
    gz_bridge_params = os.path.join(gz_pkg, 'config', 'gz_bridge.yaml')

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params}'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_image_bridge',
        arguments=['/camera/image'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    relay_camera_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        arguments=['/camera/camera_info', '/camera/image/camera_info'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── Step 6: Controllers — delay until plugin has started cm ───────────
    robot_controllers = os.path.join(
        desc_pkg, 'config', 'braccio_controllers.yaml'
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    arm_gripper_spawner = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'arm_controller',
                    'gripper_controller',
                    '--param-file', robot_controllers,
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        gz_resource_path,
        world_arg,
        use_sim_time_arg,

        xacro_resolver,       # resolve xacro first

        robot_state_publisher,
        gazebo,
        spawn_robot,          # spawn from file at t=5s

        gz_bridge,
        gz_image_bridge,
        relay_camera_info,

        joint_state_broadcaster_spawner,  # t=12s
        arm_gripper_spawner,              # t=14s
    ])