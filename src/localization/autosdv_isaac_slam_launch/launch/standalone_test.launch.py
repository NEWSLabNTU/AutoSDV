#!/usr/bin/env python3
"""
Standalone testing launch file for Isaac Visual SLAM with ZED X Mini camera.

This launch file starts:
1. ZED X Mini camera driver
2. Isaac ROS Visual SLAM with image converters and relays
3. RViz with pre-configured visualization

Usage:
    ros2 launch autosdv_isaac_slam_launch standalone_test.launch.py

Arguments:
    enable_imu_fusion:=true/false    - Enable/disable IMU fusion (default: true)
    enable_visualization:=true/false - Enable/disable SLAM visualization (default: true)
    enable_rviz:=true/false          - Start RViz (default: true)
    camera_model:=zedxm              - ZED camera model (default: zedxm)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    enable_imu_fusion_arg = DeclareLaunchArgument(
        'enable_imu_fusion',
        default_value='true',
        description='Enable IMU fusion in Visual SLAM'
    )

    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable SLAM visualization topics'
    )

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz with pre-configured displays'
    )

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zedxm',
        description='ZED camera model (zedxm, zed2i, etc.)'
    )

    # Get launch configurations
    enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_rviz = LaunchConfiguration('enable_rviz')
    camera_model = LaunchConfiguration('camera_model')

    # Get package directories
    zed_wrapper_share = get_package_share_directory('zed_wrapper')
    isaac_slam_launch_share = get_package_share_directory('autosdv_isaac_slam_launch')

    # 1. Launch ZED Camera Driver
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name': camera_model,
            'node_name': 'zed_node',
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'publish_map_tf': 'false',  # Let Isaac SLAM handle map TF
            'xacro_path': PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'urdf', 'zed_descr.urdf.xacro'
            ]),
        }.items()
    )

    # 2. Launch Isaac Visual SLAM with converters and relays
    isaac_slam_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(isaac_slam_launch_share, 'launch', 'isaac_slam_with_zed.launch.xml')
        ),
        launch_arguments={
            'enable_imu_fusion': enable_imu_fusion,
            'enable_visualization': enable_visualization,
            'camera_namespace': PathJoinSubstitution([
                '/sensing/camera',
                camera_model,
                'zed_node'
            ]),
        }.items()
    )

    # 3. Launch RViz with pre-configured display settings
    rviz_config_file = os.path.join(
        isaac_slam_launch_share,
        'rviz',
        'isaac_slam_test.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(enable_rviz),
        output='screen'
    )

    return LaunchDescription([
        # Declare arguments
        enable_imu_fusion_arg,
        enable_visualization_arg,
        enable_rviz_arg,
        camera_model_arg,

        # Launch nodes
        zed_camera_launch,
        isaac_slam_launch,
        rviz_node,
    ])
