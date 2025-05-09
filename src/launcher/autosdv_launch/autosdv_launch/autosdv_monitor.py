#!/usr/bin/env python3

import sys
import time
import json
import threading
import os
from functools import partial
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import Dict, Any, Optional, List, Union, ClassVar

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rcl_interfaces.msg import Log

# Sensor imports
from sensor_msgs.msg import Imu, PointCloud2, Image, CompressedImage, NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray

# Vehicle imports
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_control_msgs.msg import Control

# Web server
from flask import Flask, render_template, jsonify
from werkzeug.serving import make_server

@dataclass
class TopicStats:
    """Data class for storing topic statistics"""
    display_name: str
    topic_type: str
    count: int = 0
    frequency: float = 0.0
    last_time: Optional[float] = None
    last_report_count: int = 0
    last_report_time: float = field(default_factory=time.time)
    status: str = "UNKNOWN"
    details: Dict[str, Any] = field(default_factory=dict)
    rate: float = 0.0
    time_since_last: Optional[float] = None
    time_since_last_str: str = "N/A"
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to a dictionary for JSON serialization"""
        return {
            'display_name': self.display_name,
            'count': self.count,
            'rate': self.rate,
            'time_since_last': self.time_since_last_str,
            'status': self.status,
            'details': self.details
        }


class WebServerThread(threading.Thread):
    """Thread class that runs a Flask web server."""
    
    def __init__(self, node, port=8080):
        super().__init__()
        self.daemon = True  # Make the thread daemon so it dies when the main thread dies
        self.node = node
        self.port = port
        
        # Get the package directory to find templates
        self.pkg_dir = os.path.dirname(os.path.abspath(__file__))
        self.template_dir = os.path.join(self.pkg_dir, 'templates')
        
        self.app = Flask(__name__, 
                         template_folder=self.template_dir)
        
        # Setup routes
        self.setup_routes()
        
    def setup_routes(self):
        """Setup routes for the Flask app."""
        
        @self.app.route('/')
        def index():
            """Serve the main HTML page."""
            return render_template('monitor.html', 
                                 title="AutoSDV Sensor Monitor",
                                 node_name=self.node.get_name())
        
        @self.app.route('/api/status')
        def status():
            """Return the current status of all topics as JSON."""
            return jsonify(self.node.get_current_status())
            
    def run(self):
        """Run the Flask app."""
        self.server = make_server('0.0.0.0', self.port, self.app)
        self.node.get_logger().info(f"Web server running at http://localhost:{self.port}")
        self.server.serve_forever()
        
    def shutdown(self):
        """Shutdown the Flask app."""
        if hasattr(self, 'server'):
            self.server.shutdown()


class AutoSDVMonitor(Node):
    """
    A ROS2 node that monitors various sensor and vehicle interface topics
    and provides a web interface to view their status.
    """
    
    def __init__(self):
        super().__init__('autosdv_monitor')
        
        # Use a callback group to allow concurrent processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Dictionary to track message statistics
        self.topics_stats: Dict[str, TopicStats] = {}
        
        # Dictionary to track diagnostics
        self.diagnostics: Dict[str, Dict[str, Any]] = {}
        
        # Define all possible topics to monitor, organized by category
        # Each entry format: [message_type, topic_name, display_name, category_param_name]
        self.monitor_topics = {
            'lidar': [
                [PointCloud2, '/lidar/points_raw', 'Main LiDAR Raw Points', 'monitor_lidar'],
                [PointCloud2, '/sensing/lidar/concatenated/pointcloud', 'Concatenated LiDAR', 'monitor_lidar'],
                [PointCloud2, '/points', 'Blickfeld Cube1 Points', 'monitor_lidar'],
                [PointCloud2, '/lidar_front/points_raw', 'Robin-W Points', 'monitor_lidar'],
                [PointCloud2, '/velodyne_points', 'Velodyne Points', 'monitor_lidar'],
            ],
            'camera': [
                [Image, '/camera/zedxm/rgb/image_rect_color', 'ZED RGB Image', 'monitor_camera'],
                [Image, '/camera/zedxm/depth/depth_registered', 'ZED Depth Image', 'monitor_camera'],
                [Image, '/sensing/camera/traffic_light/image_raw', 'Traffic Light Camera', 'monitor_camera'],
                [CompressedImage, '/sensing/camera/traffic_light/image_raw/compressed', 'Traffic Light Camera (Compressed)', 'monitor_camera'],
            ],
            'imu': [
                [Imu, '/sensing/imu/imu_data', 'Main IMU Data', 'monitor_imu'],
                [Imu, '/imu/data_raw', 'MPU9250 Raw Data', 'monitor_imu'],
                [Imu, '/imu/data', 'MPU9250 Processed Data', 'monitor_imu'],
            ],
            'gnss': [
                [PoseStamped, '/sensing/gnss/pose', 'GNSS Pose', 'monitor_gps'],
                [PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', 'GNSS Pose with Covariance', 'monitor_gps'],
                [NavSatFix, '/sensing/gnss/ublox/nav_sat_fix', 'u-blox NavSatFix', 'monitor_gps'],
                [NavSatFix, '/sensing/gnss/garmin/nav_sat_fix', 'Garmin NavSatFix', 'monitor_gps'],
                [NavSatFix, '/sensing/gnss/septentrio/nav_sat_fix', 'Septentrio NavSatFix', 'monitor_gps'],
            ],
            'vehicle': [
                [VelocityReport, '/vehicle/status/velocity_status', 'Vehicle Velocity', 'monitor_vehicle'],
                [Control, '/control/trajectory_follower/control_cmd', 'Control Command', 'monitor_vehicle'],
                [Odometry, '/vehicle/odometry', 'Vehicle Odometry', 'monitor_vehicle'],
            ],
            'diagnostics': [
                [DiagnosticArray, '/diagnostics', 'System Diagnostics', 'monitor_diagnostics'],
                [DiagnosticArray, '/diagnostics_agg', 'Aggregated Diagnostics', 'monitor_diagnostics'],
            ],
            'system': [
                [Log, '/rosout', 'System Log', ''],  # Empty string means no parameter control, always enabled
            ]
        }
        
        # Create parameters for which topics to monitor
        self.declare_parameter('monitor_lidar', True)
        self.declare_parameter('monitor_camera', True)
        self.declare_parameter('monitor_imu', True)
        self.declare_parameter('monitor_gps', True)
        self.declare_parameter('monitor_vehicle', True)
        self.declare_parameter('monitor_diagnostics', True)
        self.declare_parameter('report_interval_sec', 5.0)
        self.declare_parameter('web_server_port', 8080)
        
        # Subscribe to topics based on parameters
        self._setup_subscriptions()
        
        # Create a timer for periodic status updates
        report_interval = self.get_parameter('report_interval_sec').value
        self.report_timer = self.create_timer(
            report_interval,
            self.update_status,
            callback_group=self.callback_group
        )
        
        # Start the web server
        port = self.get_parameter('web_server_port').value
        self.web_server = WebServerThread(self, port)
        self.web_server.start()
        
        self.get_logger().info('AutoSDV Monitor started')
        self.get_logger().info(f'Monitoring topics with report interval of {report_interval} seconds')
        self.get_logger().info(f'Web interface available at: http://localhost:{port}')
        
    def _setup_subscriptions(self):
        """Create subscriptions for all topics based on parameters"""
        # Iterate through all topic categories and topics
        for category, topics in self.monitor_topics.items():
            for topic_info in topics:
                msg_type, topic_name, display_name, param_name = topic_info
                
                # Check if this topic should be monitored
                # Empty param_name means always monitor (e.g., system topics)
                should_monitor = True
                if param_name:
                    should_monitor = self.get_parameter(param_name).value
                
                if should_monitor:
                    self._create_subscription(
                        msg_type,
                        topic_name,
                        display_name,
                        self.callback_group
                    )
                    self.get_logger().debug(f"Monitoring topic: {topic_name} ({display_name})")
        
    def _create_subscription(self, msg_type, topic, display_name, callback_group):
        """Helper to create a subscription and initialize its statistics"""
        topic_type = self._get_topic_type(topic, display_name)
        self.topics_stats[topic] = TopicStats(
            display_name=display_name,
            topic_type=topic_type,
            last_report_time=time.time()
        )
        
        return self.create_subscription(
            msg_type,
            topic,
            partial(self.topic_callback, topic_name=topic),
            10,
            callback_group=callback_group
        )
    
    def _get_topic_type(self, topic, display_name):
        """Determine the topic type based on its name or display name."""
        if 'points' in topic or 'lidar' in topic:
            return 'LiDAR Sensors'
        elif 'camera' in topic or 'image' in topic:
            return 'Camera Sensors'
        elif 'imu' in topic:
            return 'IMU Sensors'
        elif 'gnss' in topic or 'nav_sat_fix' in topic:
            return 'GNSS/GPS'
        elif 'vehicle' in topic or 'control' in topic or 'odometry' in topic:
            return 'Vehicle'
        elif 'diagnostics' in topic:
            return 'Diagnostics'
        else:
            return 'System'
    
    def topic_callback(self, msg, topic_name):
        """Generic callback for all subscribed topics"""
        stats = self.topics_stats[topic_name]
        stats.count += 1
        
        # Calculate time between messages if we've received more than one
        now = time.time()
        if stats.last_time is not None:
            delta = now - stats.last_time
            if delta > 0:
                # Exponential moving average for smoothing
                alpha = 0.3  # Smoothing factor
                new_freq = 1.0 / delta
                stats.frequency = alpha * new_freq + (1 - alpha) * stats.frequency
        
        stats.last_time = now
        stats.status = 'OK'  # Update status to OK since we received a message
        
        # Special handling for different message types
        try:
            if topic_name.endswith('/imu_data') or topic_name.endswith('/data'):
                self.process_imu(msg, stats)
            elif 'gnss' in topic_name and topic_name.endswith('nav_sat_fix'):
                self.process_gps(msg, stats)
            elif topic_name == '/vehicle/status/velocity_status':
                self.process_velocity(msg, stats)
            elif 'points' in topic_name:
                self.process_pointcloud(msg, stats, topic_name)
            elif 'image' in topic_name:
                self.process_image(msg, stats, topic_name)
            elif topic_name.endswith('/diagnostics') or topic_name.endswith('/diagnostics_agg'):
                self.process_diagnostics(msg, stats)
        except Exception as e:
            self.get_logger().warn(f"Error processing {topic_name}: {str(e)}")
            if not hasattr(stats, 'details'):
                stats.details = {}
            stats.details['error'] = str(e)
    
    def process_imu(self, msg, stats):
        """Process IMU data for additional metrics"""
        # Extract angular velocity magnitude for basic motion detection
        ang_vel = msg.angular_velocity
        magnitude = (ang_vel.x**2 + ang_vel.y**2 + ang_vel.z**2)**0.5
        
        stats.details['angular_velocity_magnitude'] = magnitude
        stats.details['linear_acceleration_magnitude'] = (
            (msg.linear_acceleration.x**2 + 
             msg.linear_acceleration.y**2 + 
             msg.linear_acceleration.z**2)**0.5
        )
        
        if magnitude > 0.1:  # Simple threshold to detect motion
            stats.details['motion_detected'] = True
            self.get_logger().debug(f'IMU motion detected: {magnitude:.3f} rad/s')
        else:
            stats.details['motion_detected'] = False
    
    def process_gps(self, msg, stats):
        """Process GPS data for quality metrics"""
        if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
            if not (msg.latitude == 0.0 and msg.longitude == 0.0):
                quality = "Good" if msg.position_covariance[0] < 100.0 else "Poor"
                stats.details['latitude'] = msg.latitude
                stats.details['longitude'] = msg.longitude
                stats.details['quality'] = quality
                stats.details['fix_type'] = getattr(msg, 'status', {}).get('status', 'Unknown')
                stats.details['satellites'] = getattr(msg, 'status', {}).get('service', 0)
                self.get_logger().debug(
                    f'GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, quality={quality}'
                )
    
    def process_velocity(self, msg, stats):
        """Process velocity report for vehicle motion detection"""
        speed = msg.longitudinal_velocity
        stats.details['speed'] = speed
        stats.details['lateral_velocity'] = msg.lateral_velocity
        stats.details['heading_rate'] = msg.heading_rate
        
        if abs(speed) > 0.05:  # Small threshold to detect vehicle movement
            stats.details['moving'] = True
            self.get_logger().debug(f'Vehicle moving at {speed:.2f} m/s')
        else:
            stats.details['moving'] = False
    
    def process_pointcloud(self, msg, stats, topic_name):
        """Process pointcloud data for basic statistics"""
        point_count = msg.width * msg.height
        stats.details['point_count'] = point_count
        stats.details['height'] = msg.height
        stats.details['width'] = msg.width
        stats.details['is_dense'] = msg.is_dense
        stats.details['point_step'] = msg.point_step
        self.get_logger().debug(f'PointCloud on {topic_name}: {point_count} points')
        
    def process_image(self, msg, stats, topic_name):
        """Process image data for basic statistics"""
        if hasattr(msg, 'width') and hasattr(msg, 'height'):
            stats.details['width'] = msg.width
            stats.details['height'] = msg.height
            stats.details['encoding'] = getattr(msg, 'encoding', 'unknown')
            stats.details['is_bigendian'] = getattr(msg, 'is_bigendian', False)
            stats.details['step'] = getattr(msg, 'step', 0)
            self.get_logger().debug(f'Image on {topic_name}: {msg.width}x{msg.height}')
    
    def process_diagnostics(self, msg, stats):
        """Process diagnostic array for device health status"""
        diag_entries = []
        
        for status in msg.status:
            level_name = "OK"
            if status.level == 1:
                level_name = "WARNING"
            elif status.level == 2:
                level_name = "ERROR"
                stats.status = 'ERROR'  # Update topic status to reflect error
            elif status.level == 3:
                level_name = "STALE"
                stats.status = 'STALE'  # Update topic status to reflect stale
                
            entry = {
                'name': status.name,
                'level': status.level,
                'level_name': level_name,
                'message': status.message,
                'hardware_id': status.hardware_id,
                'values': {value.key: value.value for value in status.values}
            }
            diag_entries.append(entry)
            
            # Store diagnostic information in a separate data structure
            key = f"{status.hardware_id}/{status.name}"
            self.diagnostics[key] = entry
            
            if status.level > 0:  # Anything above 0 indicates a warning or error
                self.get_logger().warn(f'Diagnostic {level_name} for {status.name}: {status.message}')
        
        stats.details['diagnostic_entries'] = diag_entries
    
    def update_status(self):
        """Update status for all topics."""
        now = time.time()
        
        for topic, stats in self.topics_stats.items():
            # Calculate message rate since last report
            count_delta = stats.count - stats.last_report_count
            time_delta = now - stats.last_report_time
            rate = count_delta / time_delta if time_delta > 0 else 0
            
            # Update rate information
            stats.rate = rate
            
            # Update last report values
            stats.last_report_count = stats.count
            stats.last_report_time = now
            
            # Calculate time since last message
            if stats.last_time is not None:
                stats.time_since_last = now - stats.last_time
                
                # Update status based on time since last message
                if stats.time_since_last > 10.0:  # 10 seconds threshold
                    stats.status = 'STALE'
                elif stats.count == 0:
                    stats.status = 'NO DATA'
                # Otherwise keep the current status (OK, ERROR, etc.)
            else:
                stats.time_since_last = None
                stats.status = 'NO DATA'
            
            # Format time for display
            if stats.time_since_last is not None:
                stats.time_since_last_str = f"{stats.time_since_last:.1f}s"
            else:
                stats.time_since_last_str = 'N/A'
            
            # Log status
            self.get_logger().debug(
                f"Topic: {stats.display_name} | "
                f"Rate: {rate:.1f} Hz | "
                f"Last: {stats.time_since_last_str} | "
                f"Status: {stats.status}"
            )
    
    def get_current_status(self):
        """Get the current status of all topics in a format suitable for JSON serialization."""
        # Group topics by type
        topic_groups = {}
        
        for topic, stats in sorted(self.topics_stats.items(), key=lambda x: (x[1].topic_type, x[0])):
            if stats.topic_type not in topic_groups:
                topic_groups[stats.topic_type] = []
            
            # Use the dataclass's to_dict method for serialization
            topic_groups[stats.topic_type].append(stats.to_dict())
        
        # Add system information
        system_info = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'node_name': self.get_name(),
            'topics_count': len(self.topics_stats),
            'diagnostics_count': len(self.diagnostics)
        }
        
        return {
            'system_info': system_info,
            'topic_groups': topic_groups,
            'diagnostics': self.diagnostics
        }
        
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        if hasattr(self, 'web_server'):
            self.web_server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoSDVMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
