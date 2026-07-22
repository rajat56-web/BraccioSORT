#braccio_moveit_sorting.launch.py


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution 


def generate_launch_description():

    # ── 1. Gazebo + bridges + controllers + RSP ──────────────────────────────
    gazebo_l = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('braccio_gazebo'),
                'launch', 'braccio.launch.py'
            ])
        ]),
        launch_arguments={'rviz': 'false'}.items(),
    )

    # ── 2. MoveIt move_group — delayed 5 s ───────────────────────────────────
    moveit_l = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('braccio_moveit_config'),
                        'launch', 'move_group.launch.py'
                    ])
                ]),
            )
        ],
    )

    # ── 3. YOLO HSV detector — delayed 10 s ──────────────────────────────────
    yolo_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='braccio_yolo_sorting',
                executable='yolo_detector_node',
                name='yolo_detector_node',
                output='screen',
                parameters=[{
                    'confidence_threshold': 0.4,
                    'image_topic': '/camera/image_raw',
                }],
            )
        ],
    )

    # ── 4. Sorting controller — delayed 15 s ─────────────────────────────────
    sorting_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='braccio_yolo_sorting',
                executable='braccio_moveit_sorting_controller',
                name='braccio_moveit_sorting_controller',
                output='screen',
                parameters=[{
                    'min_confidence':        0.5,
                    'detection_stable_time': 3.0,
                    'auto_start':            True,
                    'planning_group':        'arm',
                    'planning_frame':        'world',
                    # Camera intrinsics (HFOV=1.047 rad, 640x480)
                    'camera_fx': 554.4,
                    'camera_fy': 554.4,
                    'camera_cx': 320.0,
                    'camera_cy': 240.0,
                    # Camera world pose (from URDF camera_joint xyz)
                    'camera_world_x':  0.20,
                    'camera_world_y':  0.00,
                    'camera_world_z':  0.59,
                    # Known table surface height
                    'cube_world_z': 0.28,
                }],
            )
        ],
    )

    # ── 5. RViz with sorting config — delayed 5 s ────────────────────────────
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d',
                    PathJoinSubstitution([
                        FindPackageShare('braccio_description'),
                        'rviz', 'braccio_sorting.rviz'
                    ]),
                ],
            )
        ],
    )

    return LaunchDescription([
        gazebo_l,
        moveit_l,
        yolo_node,
        sorting_node,
        rviz_node,
    ])