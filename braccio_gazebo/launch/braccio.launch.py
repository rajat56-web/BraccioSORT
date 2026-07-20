#!/usr/bin/env python3
"""
It launchs Braccio in Gazebo (gz-sim) with ROS 2 Control.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    )
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------------------------------------------------------- paths
    pkg_braccio_description = get_package_share_directory('braccio_description')
    pkg_braccio_gazebo      = get_package_share_directory('braccio_gazebo')

    urdf_file   = os.path.join(pkg_braccio_description, 'urdf', 'braccio.urdf.xacro')
    world_file  = os.path.join(pkg_braccio_gazebo, 'worlds', 'braccio_sorting.world')
    rviz_config = os.path.join(pkg_braccio_description, 'rviz', 'braccio_with_camera.rviz')

    # --------------------------------------------------------------- args
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    rviz_enabled    = LaunchConfiguration('rviz')
    use_sim_time    = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------- resource path env
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_braccio_description, '..'),
    )

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib',
    )

    # ------------------------------------------------- robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )

    # -------------------------------------------- robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time':      use_sim_time,
        }],
    )

    # ---------------------------------------------------------- Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    # ------------------------------------------ spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name',  'braccio',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.0',
        ],
    )

    # ------------------------------------------ static world->base_link
    # MoveIt's virtual joint declares parent_frame="world".
    # Without this TF the IK service always returns FRAME_TRANSFORM_FAILURE.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_base',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ----------------------------------------- gz <-> ros topic bridges
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera',      '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
    )

    # ---------------------------------- load joint_state_broadcaster
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )


    # # Start controllers after robot is spawned
    # start_controllers = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=spawn_robot,
    #         on_exit=[load_jsb, load_arm, load_gripper],
    #     )
    # )

    # ------------------------------------------------------------ RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz_enabled),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        gz_resource_path,
        gz_plugin_path,
        rsp_node,
        static_tf,
        gazebo,
        spawn_robot,
        bridge,
        TimerAction(period=8.0, actions=[load_jsb]),
        TimerAction(period=10.0, actions=[load_arm]),
        TimerAction(period=10.0, actions=[load_gripper]),
        rviz_node,
    ])

