# SPDX-FileCopyrightText: AutoSDV
# SPDX-License-Identifier: Apache-2.0
"""
Visual Global Localization (cuVGL) Launch File

Launches the NVIDIA Isaac ROS Visual Global Localization node for
determining initial pose from a pre-built visual map.

Inputs:
  - Stereo camera images (mono8)
  - Camera info
  - Visual map directory

Outputs:
  - /visual_localization/pose (PoseWithCovarianceStamped)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Arguments
    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        description='Path to cuVGL visual map directory'
    )
    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value='',
        description='Path to cuVGL config (optional, uses map_dir if empty)'
    )
    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='/sensing/camera/zedxm',
        description='ZED camera namespace'
    )
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zedxm',
        description='Camera model for frame names'
    )

    # Get launch configurations
    map_dir = LaunchConfiguration('map_dir')
    config_dir = LaunchConfiguration('config_dir')
    camera_namespace = LaunchConfiguration('camera_namespace')
    camera_model = LaunchConfiguration('camera_model')

    # Derived values
    config_dir_resolved = PythonExpression([
        "'", config_dir, "' if '", config_dir, "' else '", map_dir, "'"
    ])
    left_optical_frame = PythonExpression([
        "'", camera_model, "_left_camera_optical_frame'"
    ])
    right_optical_frame = PythonExpression([
        "'", camera_model, "_right_camera_optical_frame'"
    ])

    # Composable Node Container for image format conversion
    image_converter_container = ComposableNodeContainer(
        name='visual_localization_container',
        namespace='visual_localization',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Left image format converter (RGB8 -> Mono8)
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='vgl_left_image_converter',
                parameters=[{
                    'encoding_desired': 'mono8',
                }],
                remappings=[
                    ('image_raw', [camera_namespace, '/left/color/rect/image']),
                    ('image', '/visual_localization/image_0'),
                ],
            ),
            # Right image format converter (RGB8 -> Mono8)
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='vgl_right_image_converter',
                parameters=[{
                    'encoding_desired': 'mono8',
                }],
                remappings=[
                    ('image_raw', [camera_namespace, '/right/color/rect/image']),
                    ('image', '/visual_localization/image_1'),
                ],
            ),
        ],
        output='screen',
    )

    # Camera Info Relays
    left_camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='vgl_left_camera_info_relay',
        namespace='visual_localization',
        arguments=[
            [camera_namespace, '/left/color/rect/camera_info'],
            '/visual_localization/camera_info_0'
        ],
    )

    right_camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='vgl_right_camera_info_relay',
        namespace='visual_localization',
        arguments=[
            [camera_namespace, '/right/color/rect/camera_info'],
            '/visual_localization/camera_info_1'
        ],
    )

    # Visual Global Localization Node (cuVGL)
    cuvgl_node = Node(
        package='isaac_ros_visual_global_localization',
        executable='isaac_ros_visual_global_localization',
        name='visual_global_localization_node',
        namespace='visual_localization',
        output='screen',
        parameters=[{
            # Basic configuration
            'num_cameras': 2,
            'stereo_localizer_cam_ids': '0,1',
            'image_sync_match_threshold_ms': 5.0,
            'enable_rectify_images': False,
            'enable_continuous_localization': False,
            'use_initial_guess': False,
            'localization_precision_level': 2,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'publish_map_to_base_tf': False,
            # Map paths
            'map_dir': map_dir,
            'config_dir': config_dir_resolved,
            # Camera frames
            'camera_optical_frames': [left_optical_frame, right_optical_frame],
        }],
    )

    return LaunchDescription([
        map_dir_arg,
        config_dir_arg,
        camera_namespace_arg,
        camera_model_arg,
        image_converter_container,
        left_camera_info_relay,
        right_camera_info_relay,
        cuvgl_node,
    ])
