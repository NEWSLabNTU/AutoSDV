# SPDX-FileCopyrightText: AutoSDV
# SPDX-License-Identifier: Apache-2.0
"""
Isaac ROS Visual SLAM with ZED Camera Launch File

Launches the NVIDIA Isaac ROS Visual SLAM node for continuous visual odometry.

Inputs:
  - Stereo camera images (mono8)
  - Camera info
  - IMU data (optional)

Outputs:
  - /visual_slam/tracking/odometry
  - /localization/pose_estimator/pose_with_covariance (via bridge)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('autosdv_isaac_slam_launch')

    # Arguments
    enable_imu_fusion_arg = DeclareLaunchArgument(
        'enable_imu_fusion',
        default_value='true',
        description='Enable IMU fusion in visual SLAM'
    )
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable SLAM visualization'
    )
    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='/sensing/camera/zedxm',
        description='ZED camera namespace'
    )

    # Get launch configurations
    enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')
    enable_visualization = LaunchConfiguration('enable_visualization')
    camera_namespace = LaunchConfiguration('camera_namespace')

    # Composable Node Container for image format conversion
    image_converter_container = ComposableNodeContainer(
        name='isaac_slam_image_container',
        namespace='isaac_slam',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Left image format converter (RGB8 -> Mono8)
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='left_image_converter',
                parameters=[{
                    'encoding_desired': 'mono8',
                    'image_width': 1280,
                    'image_height': 720,
                }],
                remappings=[
                    ('image_raw', [camera_namespace, '/left/color/rect/image']),
                    ('image', '/visual_slam/image_0'),
                ],
            ),
            # Right image format converter (RGB8 -> Mono8)
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='right_image_converter',
                parameters=[{
                    'encoding_desired': 'mono8',
                    'image_width': 1280,
                    'image_height': 720,
                }],
                remappings=[
                    ('image_raw', [camera_namespace, '/right/color/rect/image']),
                    ('image', '/visual_slam/image_1'),
                ],
            ),
        ],
        output='screen',
    )

    # Camera Info Relays
    left_camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='left_camera_info_relay',
        namespace='isaac_slam',
        arguments=[
            [camera_namespace, '/left/color/rect/camera_info'],
            '/visual_slam/camera_info_0'
        ],
    )

    right_camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='right_camera_info_relay',
        namespace='isaac_slam',
        arguments=[
            [camera_namespace, '/right/color/rect/camera_info'],
            '/visual_slam/camera_info_1'
        ],
    )

    # IMU Relay
    imu_relay = Node(
        package='topic_tools',
        executable='relay',
        name='imu_relay',
        namespace='isaac_slam',
        arguments=[
            [camera_namespace, '/imu/data'],
            '/visual_slam/imu'
        ],
    )

    # Isaac ROS Visual SLAM Node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam_node',
        namespace='isaac_slam',
        parameters=[
            os.path.join(pkg_share, 'config', 'isaac_slam_params.yaml'),
            {
                'enable_imu_fusion': enable_imu_fusion,
                'enable_slam_visualization': enable_visualization,
            }
        ],
    )

    # Bridge: Odometry -> PoseWithCovarianceStamped
    odometry_to_pose_bridge = Node(
        package='autosdv_isaac_slam_launch',
        executable='odometry_to_pose_bridge',
        name='isaac_slam_pose_bridge',
        namespace='isaac_slam',
        remappings=[
            ('input/odometry', '/isaac_slam/visual_slam_node/tracking/odometry'),
            ('output/pose_with_covariance', '/localization/pose_estimator/pose_with_covariance'),
        ],
    )

    # Bridge: Odometry -> TwistWithCovarianceStamped
    odometry_to_twist_bridge = Node(
        package='autosdv_isaac_slam_launch',
        executable='odometry_to_twist_bridge',
        name='isaac_slam_twist_bridge',
        namespace='isaac_slam',
        remappings=[
            ('input/odometry', '/isaac_slam/visual_slam_node/tracking/odometry'),
            ('output/twist_with_covariance', '/localization/twist_estimator/twist_with_covariance'),
        ],
    )

    return LaunchDescription([
        enable_imu_fusion_arg,
        enable_visualization_arg,
        camera_namespace_arg,
        image_converter_container,
        left_camera_info_relay,
        right_camera_info_relay,
        imu_relay,
        visual_slam_node,
        odometry_to_pose_bridge,
        odometry_to_twist_bridge,
    ])
