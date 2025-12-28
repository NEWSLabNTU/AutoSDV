#!/usr/bin/env python3
"""
Interactive TUI for Autoware autonomous driving control.
Reads poses from poses.json and provides manual control over localization,
routing, and operation mode changes.

Features:
- Real-time status display (localization, route, operation mode, position, velocity)
- NDT debug statistics (score, point count, iterations, execution time)
- Emacs-style combo keys for pose selection (e.g., "I then 2" to init with pose #2)
- Manual control of pose initialization (no automatic retries)
- Clear visual feedback with scrolling log

Controls (Emacs-style combo keys):
  [I]+[1-9]       - Initialize localization with selected initial pose
  [R]+[1-9]       - Set route to selected goal pose
  [C] Clear Route - Clear current route
  [A] Autonomous  - Engage autonomous driving
  [M] Manual      - Switch to manual control
  [Q] Quit        - Exit the program
  [ESC]           - Cancel current combo command

Example: Press "I", then "2" to initialize with the 2nd initial pose.

IMPORTANT: The initial and goal poses MUST be on connected lanes in the lanelet2 map.
Use RViz's "2D Pose Estimate" and "2D Goal Pose" to verify poses are valid.
"""

import json
import math
import time
import sys
import curses
from pathlib import Path
from datetime import datetime
from collections import deque
from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    OperationModeState,
    RouteState,
)
from autoware_adapi_v1_msgs.srv import (
    ChangeOperationMode,
    ClearRoute,
    InitializeLocalization,
    SetRoutePoints,
)
from autoware_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tier4_debug_msgs.msg import Float32Stamped, Int32Stamped


# Route state constants (from RouteState message)
class RouteStateEnum:
    UNKNOWN = 0
    UNSET = 1
    SET = 2
    ARRIVED = 3
    CHANGING = 4


# Localization state constants (from LocalizationInitializationState message)
class LocalizationStateEnum:
    UNKNOWN = 0
    UNINITIALIZED = 1
    INITIALIZING = 2
    INITIALIZED = 3


# Configuration
class Config:
    # Timeouts (seconds)
    SERVICE_TIMEOUT = 15.0
    GOAL_DISTANCE_THRESHOLD = 5.0

    # TUI settings
    LOG_MAX_LINES = 10
    STATUS_UPDATE_RATE = 0.1  # 10 Hz

    # Terminal size requirements
    MIN_WIDTH = 30
    MIN_HEIGHT = 20


def load_poses():
    """Load poses from JSON file in the same directory as this script."""
    poses_path = Path(__file__).parent / "poses.json"

    if not poses_path.exists():
        print(f"Error: poses.json not found at {poses_path}", file=sys.stderr)
        print(
            "Please run read_poses.py first to capture initial and goal poses",
            file=sys.stderr,
        )
        sys.exit(1)

    with open(poses_path, "r") as f:
        poses_data = json.load(f)

    # Handle old format (single pose pairs)
    if "initial_pose" in poses_data and "goal_pose" in poses_data:
        print("  Converting old poses.json format to new format...")
        poses_data = {
            "initial_poses": [
                {
                    "name": "Start 1",
                    "pose": poses_data["initial_pose"]
                }
            ],
            "goal_poses": [
                {
                    "name": "Goal 1",
                    "pose": poses_data["goal_pose"]
                }
            ]
        }

    # Validate new format
    if "initial_poses" not in poses_data or "goal_poses" not in poses_data:
        print("Error: Invalid poses.json format", file=sys.stderr)
        sys.exit(1)

    if not poses_data["initial_poses"] or not poses_data["goal_poses"]:
        print("Error: poses.json contains no poses", file=sys.stderr)
        print("Please run read_poses.py to capture poses", file=sys.stderr)
        sys.exit(1)

    print(f"  Loaded {len(poses_data['initial_poses'])} initial poses and {len(poses_data['goal_poses'])} goal poses")
    return poses_data


def calculate_distance(x1, y1, x2, y2):
    """Calculate 2D distance between two points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class AutowareDriveNode(Node):
    """ROS 2 node for autonomous driving with Autoware TUI."""

    def __init__(self, poses_data):
        super().__init__("autoware_drive_node")

        # Store all available poses
        self.poses_data = poses_data
        self.current_initial_idx = 0
        self.current_goal_idx = 0

        # Thread-safe state
        self.lock = Lock()
        self.current_position = None
        self.current_velocity = 0.0
        self.route_state = None
        self.operation_mode = None
        self.localization_state = None

        # NDT debug statistics (None = no data received yet)
        self.ndt_score = None
        self.ndt_point_count = None
        self.ndt_iterations = None
        self.ndt_exe_time = None
        self.ndt_initial_to_result_distance = None
        self.ndt_map_point_count = 0  # Keep 0, shows "(loading...)"

        # TUI log buffer
        self.log_buffer = deque(maxlen=Config.LOG_MAX_LINES)

        # QoS profiles
        qos_transient_local = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Service clients - Modern Autoware API
        self.init_localization_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )

        self.set_route_client = self.create_client(
            SetRoutePoints, "/api/routing/set_route_points"
        )

        self.clear_route_client = self.create_client(
            ClearRoute, "/api/routing/clear_route"
        )

        self.change_mode_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )

        # Subscribers
        self.velocity_sub = self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.velocity_callback,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.odometry_callback,
            10,
        )

        self.route_state_sub = self.create_subscription(
            RouteState,
            "/api/routing/state",
            self.route_state_callback,
            qos_transient_local,
        )

        self.operation_mode_sub = self.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self.operation_mode_callback,
            qos_transient_local,
        )

        self.localization_state_sub = self.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            self.localization_state_callback,
            qos_transient_local,
        )

        # NDT debug subscribers
        self.ndt_score_sub = self.create_subscription(
            Float32Stamped,
            "/localization/pose_estimator/nearest_voxel_transformation_likelihood",
            self.ndt_score_callback,
            10,
        )

        self.ndt_points_sub = self.create_subscription(
            PointCloud2,
            "/localization/pose_estimator/points_aligned",
            self.ndt_points_callback,
            10,
        )

        self.ndt_iterations_sub = self.create_subscription(
            Int32Stamped,
            "/localization/pose_estimator/iteration_num",
            self.ndt_iterations_callback,
            10,
        )

        self.ndt_exe_time_sub = self.create_subscription(
            Float32Stamped,
            "/localization/pose_estimator/exe_time_ms",
            self.ndt_exe_time_callback,
            10,
        )

        self.ndt_distance_sub = self.create_subscription(
            Float32Stamped,
            "/localization/pose_estimator/initial_to_result_distance",
            self.ndt_distance_callback,
            10,
        )

        # Map points subscriber with TRANSIENT_LOCAL QoS (published once at startup)
        qos_transient_map = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ndt_map_points_sub = self.create_subscription(
            PointCloud2,
            "/localization/pose_estimator/debug/loaded_pointcloud_map",
            self.ndt_map_points_callback,
            qos_transient_map,
        )

        self.log("Autoware Drive Node initialized")

    def add_log(self, message):
        """Add a message to the log buffer (thread-safe)."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        with self.lock:
            self.log_buffer.append(f"[{timestamp}] {message}")

    def log(self, message):
        """Log message to both ROS logger and TUI buffer."""
        self.get_logger().info(message)
        self.add_log(message)

    def velocity_callback(self, msg):
        """Update current velocity."""
        with self.lock:
            self.current_velocity = msg.longitudinal_velocity

    def odometry_callback(self, msg):
        """Update current position from odometry."""
        with self.lock:
            self.current_position = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
            }

    def route_state_callback(self, msg):
        """Update route state."""
        with self.lock:
            self.route_state = msg.state

    def operation_mode_callback(self, msg):
        """Update operation mode."""
        with self.lock:
            self.operation_mode = msg.mode

    def localization_state_callback(self, msg):
        """Update localization initialization state."""
        with self.lock:
            self.localization_state = msg.state

    def ndt_score_callback(self, msg):
        """Update NDT matching score."""
        with self.lock:
            self.ndt_score = msg.data

    def ndt_points_callback(self, msg):
        """Update NDT point count."""
        with self.lock:
            # Point cloud: width * height = total points
            self.ndt_point_count = msg.width * msg.height

    def ndt_iterations_callback(self, msg):
        """Update NDT iteration count."""
        with self.lock:
            self.ndt_iterations = msg.data

    def ndt_exe_time_callback(self, msg):
        """Update NDT execution time."""
        with self.lock:
            self.ndt_exe_time = msg.data

    def ndt_distance_callback(self, msg):
        """Update NDT initial to result distance."""
        with self.lock:
            self.ndt_initial_to_result_distance = msg.data

    def ndt_map_points_callback(self, msg):
        """Update NDT map point count (published once at startup)."""
        with self.lock:
            self.ndt_map_point_count = msg.width * msg.height

    def get_state_snapshot(self):
        """Get a thread-safe snapshot of current state."""
        with self.lock:
            return {
                'position': self.current_position.copy() if self.current_position else None,
                'velocity': self.current_velocity,
                'route_state': self.route_state,
                'operation_mode': self.operation_mode,
                'localization_state': self.localization_state,
                'ndt_score': self.ndt_score,
                'ndt_point_count': self.ndt_point_count,
                'ndt_iterations': self.ndt_iterations,
                'ndt_exe_time': self.ndt_exe_time,
                'ndt_distance': self.ndt_initial_to_result_distance,
                'ndt_map_points': self.ndt_map_point_count,
                'log_lines': list(self.log_buffer)
            }

    def spin_once_nonblocking(self):
        """Spin once without blocking."""
        rclpy.spin_once(self, timeout_sec=0.0)

    def call_service(self, client, request, service_name, timeout_sec=None):
        """Call a service (single attempt, no retry)."""
        if timeout_sec is None:
            timeout_sec = Config.SERVICE_TIMEOUT

        # Wait for service
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.log(f"ERROR: {service_name} service not available")
            return False, None

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is not None:
            result = future.result()
            if result.status.success:
                return True, result
            else:
                self.log(f"ERROR: {service_name} - {result.status.message}")
                return False, result
        else:
            self.log(f"ERROR: {service_name} call timed out")
            return False, None

    def initialize_localization(self, pose_idx=None):
        """Initialize localization using the Autoware API service."""
        if pose_idx is None:
            pose_idx = self.current_initial_idx

        if pose_idx >= len(self.poses_data["initial_poses"]):
            self.log(f"ERROR: Invalid pose index {pose_idx}")
            return False

        # Update current index
        self.current_initial_idx = pose_idx

        # Get the selected pose
        pose_entry = self.poses_data["initial_poses"][pose_idx]
        initial_pose = pose_entry["pose"]

        self.log(f"Initializing with '{pose_entry['name']}'...")

        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = initial_pose["position"]["x"]
        pose_msg.pose.pose.position.y = initial_pose["position"]["y"]
        pose_msg.pose.pose.position.z = initial_pose["position"]["z"]

        pose_msg.pose.pose.orientation.x = initial_pose["orientation"]["x"]
        pose_msg.pose.pose.orientation.y = initial_pose["orientation"]["y"]
        pose_msg.pose.pose.orientation.z = initial_pose["orientation"]["z"]
        pose_msg.pose.pose.orientation.w = initial_pose["orientation"]["w"]

        # Set covariance (small values indicate high confidence)
        pose_msg.pose.covariance[0] = 0.25  # x variance
        pose_msg.pose.covariance[7] = 0.25  # y variance
        pose_msg.pose.covariance[35] = 0.06853891909122467  # yaw variance

        # Call service
        request = InitializeLocalization.Request()
        request.pose.append(pose_msg)

        success, _ = self.call_service(
            self.init_localization_client,
            request,
            "Initialize localization",
        )

        if success:
            self.log(f"Localization initialized at ({initial_pose['position']['x']:.1f}, {initial_pose['position']['y']:.1f})")

        return success

    def clear_route(self):
        """Clear existing route."""
        self.log("Clearing route...")
        request = ClearRoute.Request()
        success, _ = self.call_service(
            self.clear_route_client,
            request,
            "Clear route",
        )
        if success:
            self.log("Route cleared")
        return success

    def set_route(self, goal_idx=None):
        """Set route to goal pose."""
        if goal_idx is None:
            goal_idx = self.current_goal_idx

        if goal_idx >= len(self.poses_data["goal_poses"]):
            self.log(f"ERROR: Invalid goal index {goal_idx}")
            return False

        # Update current index
        self.current_goal_idx = goal_idx

        # Get the selected goal pose
        goal_entry = self.poses_data["goal_poses"][goal_idx]
        goal_pose = goal_entry["pose"]

        self.log(f"Setting route to '{goal_entry['name']}'...")

        request = SetRoutePoints.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "map"

        # Set goal pose
        goal = Pose()
        goal.position.x = goal_pose["position"]["x"]
        goal.position.y = goal_pose["position"]["y"]
        goal.position.z = goal_pose["position"]["z"]
        goal.orientation.x = goal_pose["orientation"]["x"]
        goal.orientation.y = goal_pose["orientation"]["y"]
        goal.orientation.z = goal_pose["orientation"]["z"]
        goal.orientation.w = goal_pose["orientation"]["w"]

        request.goal = goal
        request.option.allow_goal_modification = True

        success, _ = self.call_service(
            self.set_route_client,
            request,
            "Set route",
        )

        if success:
            self.log(f"Route set to goal ({goal_pose['position']['x']:.1f}, {goal_pose['position']['y']:.1f})")

        return success

    def set_autonomous_mode(self):
        """Change to autonomous operation mode."""
        self.log("Engaging autonomous mode...")
        request = ChangeOperationMode.Request()
        success, _ = self.call_service(
            self.change_mode_client,
            request,
            "Change to autonomous mode",
        )
        if success:
            self.log("Autonomous mode engaged")
        return success

    def set_manual_mode(self):
        """Change to manual operation mode."""
        self.log("Switching to manual mode...")
        # Use the stop service instead
        client = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_stop")
        request = ChangeOperationMode.Request()
        success, _ = self.call_service(
            client,
            request,
            "Change to manual mode",
        )
        if success:
            self.log("Manual mode activated")
        return success


class PoseSelector:
    """TUI for selecting initial and goal poses."""

    def __init__(self, stdscr, poses_data):
        self.stdscr = stdscr
        self.poses_data = poses_data
        self.selected_initial_idx = 0
        self.selected_goal_idx = 0
        self.current_section = 0  # 0=initial, 1=goal
        self.confirmed = False

        # Setup curses
        curses.curs_set(0)
        stdscr.nodelay(0)  # Blocking input for selection

        # Initialize color pairs
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)   # Selected
        curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)  # Current section
        curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)    # Headers
        curses.init_pair(4, curses.COLOR_WHITE, curses.COLOR_BLACK)   # Normal

    def draw_header(self, y):
        """Draw header."""
        title = "POSE SELECTION"
        width = self.stdscr.getmaxyx()[1]
        self.stdscr.addstr(y, 0, "=" * width)
        self.stdscr.addstr(y + 1, (width - len(title)) // 2, title, curses.color_pair(3) | curses.A_BOLD)
        self.stdscr.addstr(y + 2, 0, "=" * width)
        return y + 3

    def draw_pose_list(self, y, poses, selected_idx, title, is_current):
        """Draw a list of poses with selection."""
        # Title
        title_color = curses.color_pair(2) if is_current else curses.color_pair(4)
        self.stdscr.addstr(y, 0, title, title_color | curses.A_BOLD)
        y += 1

        # Poses
        for i, pose_entry in enumerate(poses):
            pose_data = pose_entry["pose"]
            name = pose_entry["name"]
            x = pose_data["position"]["x"]
            y_pos = pose_data["position"]["y"]

            # Format line
            prefix = "> " if i == selected_idx else "  "
            line = f"{prefix}[{i+1}] {name:15s}  ({x:7.2f}, {y_pos:7.2f})"

            # Color
            if i == selected_idx:
                color = curses.color_pair(1) | curses.A_BOLD
            else:
                color = curses.color_pair(4)

            self.stdscr.addstr(y, 2, line, color)
            y += 1

        return y + 1

    def draw_controls(self, y):
        """Draw control instructions."""
        width = self.stdscr.getmaxyx()[1]
        self.stdscr.addstr(y, 0, "-" * width)
        y += 1
        self.stdscr.addstr(y, 0, "CONTROLS", curses.color_pair(3) | curses.A_BOLD)
        y += 1
        self.stdscr.addstr(y, 2, "[Up/Down] or [J/K] Navigate   [Tab] Switch section   [Enter] Confirm")
        y += 1
        self.stdscr.addstr(y, 2, "[1-9]  Jump to pose      [Q] Quit")
        return y + 1

    def update_display(self):
        """Update the entire display."""
        self.stdscr.clear()

        y = 0
        y = self.draw_header(y)

        # Draw initial poses
        y = self.draw_pose_list(
            y,
            self.poses_data["initial_poses"],
            self.selected_initial_idx,
            "INITIAL POSES",
            self.current_section == 0
        )

        # Draw goal poses
        y = self.draw_pose_list(
            y,
            self.poses_data["goal_poses"],
            self.selected_goal_idx,
            "GOAL POSES",
            self.current_section == 1
        )

        # Draw controls
        self.draw_controls(y)

        # Draw current selection summary
        init_pose = self.poses_data["initial_poses"][self.selected_initial_idx]
        goal_pose = self.poses_data["goal_poses"][self.selected_goal_idx]
        summary_y = self.stdscr.getmaxyx()[0] - 4
        if summary_y > y:
            width = self.stdscr.getmaxyx()[1]
            self.stdscr.addstr(summary_y, 0, "-" * width)
            self.stdscr.addstr(summary_y + 1, 0, "SELECTED:", curses.color_pair(3))
            self.stdscr.addstr(
                summary_y + 2, 2,
                f"Initial: {init_pose['name']:15s}  Goal: {goal_pose['name']}"
            )

        self.stdscr.refresh()

    def handle_input(self, key):
        """Handle keyboard input."""
        if key == ord('q') or key == ord('Q'):
            return False  # Quit

        elif key == ord('\n'):  # Enter
            self.confirmed = True
            return False  # Exit selection

        elif key == ord('\t'):  # Tab - switch section
            self.current_section = 1 - self.current_section

        elif key == curses.KEY_UP or key == ord('k') or key == ord('K'):
            # Move up
            if self.current_section == 0:
                self.selected_initial_idx = max(0, self.selected_initial_idx - 1)
            else:
                self.selected_goal_idx = max(0, self.selected_goal_idx - 1)

        elif key == curses.KEY_DOWN or key == ord('j') or key == ord('J'):
            # Move down
            if self.current_section == 0:
                max_idx = len(self.poses_data["initial_poses"]) - 1
                self.selected_initial_idx = min(max_idx, self.selected_initial_idx + 1)
            else:
                max_idx = len(self.poses_data["goal_poses"]) - 1
                self.selected_goal_idx = min(max_idx, self.selected_goal_idx + 1)

        elif ord('1') <= key <= ord('9'):
            # Jump to pose by number
            idx = key - ord('1')
            if self.current_section == 0:
                if idx < len(self.poses_data["initial_poses"]):
                    self.selected_initial_idx = idx
            else:
                if idx < len(self.poses_data["goal_poses"]):
                    self.selected_goal_idx = idx

        return True  # Continue

    def run(self):
        """Main selection loop."""
        while True:
            self.update_display()
            key = self.stdscr.getch()
            if not self.handle_input(key):
                break

        return self.confirmed, self.selected_initial_idx, self.selected_goal_idx


class AutowareTUI:
    """Interactive TUI for Autoware control."""

    def __init__(self, stdscr, node):
        self.stdscr = stdscr
        self.node = node
        self.running = True

        # Command buffer for Emacs-style combo keys
        self.command_buffer = ""
        self.waiting_for_number = False
        self.current_command = None  # 'I' or 'R'

        # Setup curses
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(1)   # Non-blocking input
        stdscr.timeout(100)  # 100ms refresh

        # Initialize color pairs
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)   # Status OK
        curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)  # Status Warning
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)     # Status Error
        curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)    # Headers
        curses.init_pair(5, curses.COLOR_MAGENTA, curses.COLOR_BLACK)  # Command buffer

    def check_terminal_size(self):
        """Check if terminal is large enough."""
        height, width = self.stdscr.getmaxyx()
        return height >= Config.MIN_HEIGHT and width >= Config.MIN_WIDTH

    def draw_terminal_too_small(self):
        """Show message when terminal is too small."""
        try:
            self.stdscr.clear()
            height, width = self.stdscr.getmaxyx()

            # Adapt message to very small terminals
            if width >= 40:
                msg_lines = [
                    "TERMINAL TOO SMALL",
                    "",
                    f"Current: {width}x{height}",
                    f"Min: {Config.MIN_WIDTH}x{Config.MIN_HEIGHT}",
                    "",
                    "Resize or press Q"
                ]
            elif width >= 25:
                msg_lines = [
                    "TOO SMALL",
                    "",
                    f"{width}x{height}",
                    f"Min:{Config.MIN_WIDTH}x{Config.MIN_HEIGHT}",
                    "",
                    "Resize or Q"
                ]
            else:
                msg_lines = [
                    "TOO SMALL",
                    f"{width}x{height}",
                    f"{Config.MIN_WIDTH}x{Config.MIN_HEIGHT}",
                    "Q=quit"
                ]

            start_y = max(0, (height - len(msg_lines)) // 2)

            for i, line in enumerate(msg_lines):
                y = start_y + i
                if y >= height:
                    break
                # Center if possible, otherwise left-align
                if len(line) < width:
                    x = max(0, (width - len(line)) // 2)
                else:
                    x = 0
                # Truncate line if too long
                display_line = line[:width] if len(line) > width else line
                if x < width and len(display_line) > 0:
                    self.stdscr.addstr(y, x, display_line, curses.color_pair(3) | curses.A_BOLD)

            self.stdscr.refresh()
        except curses.error:
            # Ignore any curses errors during drawing
            pass

    def get_localization_state_str(self, state):
        """Convert localization state to string."""
        states = {
            LocalizationStateEnum.UNKNOWN: ("UNKNOWN", 2),
            LocalizationStateEnum.UNINITIALIZED: ("UNINITIALIZED", 2),
            LocalizationStateEnum.INITIALIZING: ("INITIALIZING", 2),
            LocalizationStateEnum.INITIALIZED: ("INITIALIZED", 1),
        }
        return states.get(state, (f"STATE_{state}", 3))

    def get_route_state_str(self, state):
        """Convert route state to string."""
        states = {
            RouteStateEnum.UNKNOWN: ("UNKNOWN", 2),
            RouteStateEnum.UNSET: ("UNSET", 2),
            RouteStateEnum.SET: ("SET", 1),
            RouteStateEnum.ARRIVED: ("ARRIVED", 1),
            RouteStateEnum.CHANGING: ("CHANGING", 2),
        }
        return states.get(state, (f"STATE_{state}", 3))

    def get_operation_mode_str(self, mode):
        """Convert operation mode to string."""
        # From OperationModeState.msg
        modes = {
            1: ("STOP", 2),
            2: ("AUTONOMOUS", 1),
            3: ("LOCAL", 2),
            4: ("REMOTE", 2),
        }
        return modes.get(mode, ("UNKNOWN", 3))

    def draw_header(self, y):
        """Draw header."""
        try:
            height, width = self.stdscr.getmaxyx()
            if y + 2 >= height:
                return y + 3

            # Adapt title to terminal width
            if width >= 42:
                title = "AUTOWARE AUTONOMOUS DRIVING CONTROLLER"
            elif width >= 25:
                title = "AUTOWARE CONTROLLER"
            else:
                title = "AUTOWARE"

            # Draw separator
            self.stdscr.addstr(y, 0, "=" * width)
            # Draw title (centered, truncated if needed)
            if len(title) < width:
                x = max(0, (width - len(title)) // 2)
                self.stdscr.addstr(y + 1, x, title, curses.color_pair(4) | curses.A_BOLD)
            else:
                self.stdscr.addstr(y + 1, 0, title[:width], curses.color_pair(4) | curses.A_BOLD)
            # Draw separator
            self.stdscr.addstr(y + 2, 0, "=" * width)
        except curses.error:
            pass
        return y + 3

    def draw_status(self, y, state):
        """Draw status section."""
        try:
            height, width = self.stdscr.getmaxyx()
            if y >= height:
                return y + 1

            self.stdscr.addstr(y, 0, "STATUS", curses.color_pair(4) | curses.A_BOLD)
            y += 1

            # Adapt labels based on width
            if width >= 50:
                loc_label = "Localization: "
                route_label = "Route:        "
                mode_label = "Op Mode:      "
                pos_label = "Position:     "
                vel_label = "Velocity:     "
                goal_label = "Goal Dist:    "
            elif width >= 35:
                loc_label = "Loc: "
                route_label = "Route: "
                mode_label = "Mode:  "
                pos_label = "Pos:   "
                vel_label = "Vel:   "
                goal_label = "Goal:  "
            else:
                loc_label = "L:"
                route_label = "R:"
                mode_label = "M:"
                pos_label = "P:"
                vel_label = "V:"
                goal_label = "G:"

            # Localization
            if y < height:
                loc_str, loc_color = self.get_localization_state_str(state['localization_state'])
                line = f"{loc_label}{loc_str}"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, loc_label)
                self.stdscr.addstr(loc_str[:width - 2 - len(loc_label)], curses.color_pair(loc_color))
            y += 1

            # Route
            if y < height:
                route_str, route_color = self.get_route_state_str(state['route_state'])
                line = f"{route_label}{route_str}"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, route_label)
                self.stdscr.addstr(route_str[:width - 2 - len(route_label)], curses.color_pair(route_color))
            y += 1

            # Operation Mode
            if y < height:
                mode_str, mode_color = self.get_operation_mode_str(state['operation_mode'])
                line = f"{mode_label}{mode_str}"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, mode_label)
                self.stdscr.addstr(mode_str[:width - 2 - len(mode_label)], curses.color_pair(mode_color))
            y += 1

            # Position
            if y < height:
                if state['position']:
                    pos = state['position']
                    line = f"{pos_label}({pos['x']:.1f},{pos['y']:.1f})"
                else:
                    line = f"{pos_label}--"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, line)
            y += 1

            # Velocity
            if y < height:
                line = f"{vel_label}{state['velocity']:.1f}m/s"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, line)
            y += 1

            # Current goal distance
            if y < height:
                goal_idx = self.node.current_goal_idx
                if goal_idx < len(self.node.poses_data["goal_poses"]):
                    goal_entry = self.node.poses_data["goal_poses"][goal_idx]
                    goal_pos = goal_entry["pose"]["position"]
                    if state['position']:
                        pos = state['position']
                        dist = calculate_distance(pos['x'], pos['y'], goal_pos['x'], goal_pos['y'])
                        if width >= 50:
                            line = f"{goal_label}{dist:.1f}m ({goal_entry['name']})"
                        else:
                            line = f"{goal_label}{dist:.1f}m"
                    else:
                        line = f"{goal_label}--"
                else:
                    line = f"{goal_label}--"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, line)
            y += 1
        except curses.error:
            pass

        # Blank line before NDT stats
        y += 1

        # NDT Debug Statistics
        try:
            if y < height:
                title = "NDT" if width < 35 else "NDT DEBUG"
                self.stdscr.addstr(y, 0, title, curses.color_pair(4) | curses.A_BOLD)
            y += 1

            # Adapt labels based on width
            if width >= 50:
                score_label = "Score:        "
                pts_label = "Sensor Pts:   "
                map_label = "Map Pts:      "
                iter_label = "Iterations:   "
                time_label = "Exe Time:     "
                dist_label = "Init->Result: "
            elif width >= 35:
                score_label = "Score: "
                pts_label = "Pts:   "
                map_label = "Map:   "
                iter_label = "Iter:  "
                time_label = "Time:  "
                dist_label = "Dist:  "
            else:
                score_label = "S:"
                pts_label = "P:"
                map_label = "M:"
                iter_label = "I:"
                time_label = "T:"
                dist_label = "D:"

            # NDT Score (color-coded: green >2.0, yellow 1.5-2.0, red <1.5)
            if y < height:
                score = state['ndt_score']
                self.stdscr.addstr(y, 2, score_label)
                if score is not None:
                    score_color = 1 if score >= 2.0 else (2 if score >= 1.5 else 3)
                    val = f"{score:.3f}" if width >= 35 else f"{score:.2f}"
                    self.stdscr.addstr(val, curses.color_pair(score_color))
                else:
                    self.stdscr.addstr("--", curses.color_pair(2))
            y += 1

            # Sensor point count
            if y < height:
                points = state['ndt_point_count']
                self.stdscr.addstr(y, 2, pts_label)
                if points is not None:
                    points_color = 1 if points >= 5000 else (2 if points >= 3000 else 3)
                    val = f"{points:,}" if width >= 50 else f"{points//1000}k"
                    self.stdscr.addstr(val, curses.color_pair(points_color))
                else:
                    self.stdscr.addstr("--", curses.color_pair(2))
            y += 1

            # Map point count
            if y < height:
                map_pts = state['ndt_map_points']
                line = f"{map_label}"
                if map_pts > 0:
                    if width >= 50:
                        line += f"{map_pts:,}"
                    else:
                        line += f"{map_pts//1000}k"
                else:
                    line += "..." if width < 35 else "(loading...)"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, line)
            y += 1

            # Iterations
            if y < height:
                iters = state['ndt_iterations']
                line = f"{iter_label}"
                if iters is not None:
                    line += f"{iters}"
                else:
                    line += "--"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, line)
            y += 1

            # Execution time
            if y < height:
                exe_time = state['ndt_exe_time']
                line = f"{time_label}"
                if exe_time is not None:
                    if width >= 35:
                        line += f"{exe_time:.1f}ms"
                    else:
                        line += f"{int(exe_time)}ms"
                else:
                    line += "--"
                if len(line) > width - 2:
                    line = line[:width - 2]
                self.stdscr.addstr(y, 2, line)
            y += 1

            # Initial to result distance
            if y < height:
                distance = state['ndt_distance']
                self.stdscr.addstr(y, 2, dist_label)
                if distance is not None:
                    dist_color = 1 if distance < 0.5 else (2 if distance < 1.0 else 3)
                    val = f"{distance:.3f}m" if width >= 35 else f"{distance:.2f}m"
                    self.stdscr.addstr(val, curses.color_pair(dist_color))
                else:
                    self.stdscr.addstr("--", curses.color_pair(2))
            y += 1
        except curses.error:
            pass

        return y + 1

    def draw_commands(self, y):
        """Draw commands section."""
        try:
            height, width = self.stdscr.getmaxyx()
            if y >= height - 1:
                return y + 1
            self.stdscr.addstr(y, 0, "-" * width)
            y += 1
            if y < height:
                title = "CMD" if width < 35 else "COMMANDS"
                self.stdscr.addstr(y, 0, title, curses.color_pair(4) | curses.A_BOLD)
            y += 1

            # Show command buffer if active
            if self.waiting_for_number:
                if y < height:
                    line = f"Cmd: {self.command_buffer}" if width < 35 else f"Command: {self.command_buffer}"
                    if len(line) > width - 2:
                        line = line[:width - 2]
                    self.stdscr.addstr(y, 2, line, curses.color_pair(5) | curses.A_BOLD)
                y += 1
                if y < height:
                    if width >= 50:
                        line = "(Press 1-9 to select pose, ESC to cancel)"
                    elif width >= 35:
                        line = "1-9:select ESC:cancel"
                    else:
                        line = "1-9 ESC"
                    if len(line) > width - 2:
                        line = line[:width - 2]
                    self.stdscr.addstr(y, 2, line, curses.color_pair(2))
                y += 1
            else:
                # Adapt commands display to width
                if width >= 80:
                    if y < height:
                        self.stdscr.addstr(y, 2, "[I]+[1-9] Init Pose    [R]+[1-9] Set Route    [C] Clear Route")
                    y += 1
                    if y < height:
                        self.stdscr.addstr(y, 2, "[A] Autonomous Mode    [M] Manual Mode       [Q] Quit")
                    y += 1
                elif width >= 50:
                    if y < height:
                        self.stdscr.addstr(y, 2, "[I]+[1-9] Init  [R]+[1-9] Route  [C] Clear")
                    y += 1
                    if y < height:
                        self.stdscr.addstr(y, 2, "[A] Auto  [M] Manual  [Q] Quit")
                    y += 1
                elif width >= 35:
                    if y < height:
                        self.stdscr.addstr(y, 2, "I+# R+# C A M Q")
                    y += 1
                else:
                    # Ultra compact
                    if y < height:
                        self.stdscr.addstr(y, 2, "I# R# C A M Q")
                    y += 1
        except curses.error:
            pass
        return y + 1

    def draw_pose_selection(self, y):
        """Draw pose selection list when waiting for number input."""
        try:
            height, width = self.stdscr.getmaxyx()

            # Check if we have room to draw pose selection
            if y >= height - 3:
                return y  # Not enough space, skip pose selection

            self.stdscr.addstr(y, 0, "-" * width)
            y += 1

            if self.current_command == 'I':
                # Show initial poses
                title = "INIT" if width < 35 else "SELECT INITIAL POSE"
                if y < height:
                    self.stdscr.addstr(y, 0, title, curses.color_pair(4) | curses.A_BOLD)
                y += 1
                poses = self.node.poses_data["initial_poses"]
                current_idx = self.node.current_initial_idx
            else:  # 'R'
                # Show goal poses
                title = "GOAL" if width < 35 else "SELECT GOAL POSE"
                if y < height:
                    self.stdscr.addstr(y, 0, title, curses.color_pair(4) | curses.A_BOLD)
                y += 1
                poses = self.node.poses_data["goal_poses"]
                current_idx = self.node.current_goal_idx

            # Display poses (limit to available space)
            max_poses = min(len(poses), height - y - 2)  # Leave room for log
            for i in range(max_poses):
                if y >= height - 1:
                    break  # No more room

                pose_entry = poses[i]
                pose_data = pose_entry["pose"]
                name = pose_entry["name"]
                x = pose_data["position"]["x"]
                y_pos = pose_data["position"]["y"]

                # Indicate current selection
                prefix = "*" if i == current_idx else " "

                # Adapt format to width
                if width >= 60:
                    line = f"{prefix}[{i+1}] {name:20s}  ({x:7.2f}, {y_pos:7.2f})"
                elif width >= 40:
                    line = f"{prefix}[{i+1}] {name:12s} ({x:.1f},{y_pos:.1f})"
                else:
                    line = f"{prefix}{i+1} {name[:10]}"

                # Truncate if still too long
                if len(line) > width - 2:
                    line = line[:width - 2]

                # Color current selection
                color = curses.color_pair(1) if i == current_idx else curses.color_pair(4)
                self.stdscr.addstr(y, 2, line, color)
                y += 1
        except curses.error:
            pass

        return y + 1

    def draw_log(self, y, state):
        """Draw log section."""
        try:
            height, width = self.stdscr.getmaxyx()

            # Check if we have room to draw log section
            if y >= height - 3:
                return  # Not enough space, skip log section

            self.stdscr.addstr(y, 0, "-" * width)
            y += 1
            if y < height:
                self.stdscr.addstr(y, 0, "LOG", curses.color_pair(4) | curses.A_BOLD)
            y += 1

            # Display log lines (most recent at bottom)
            log_lines = state['log_lines']
            max_log_lines = min(height - y - 1, Config.LOG_MAX_LINES)

            for i, line in enumerate(log_lines[-max_log_lines:]):
                if y + i < height - 1:
                    # Truncate line if too long
                    if len(line) > width - 3:
                        line = line[:width - 6] + "..."
                    self.stdscr.addstr(y + i, 2, line)
        except curses.error:
            pass

    def update_display(self):
        """Update the entire display."""
        # Check terminal size first
        if not self.check_terminal_size():
            self.draw_terminal_too_small()
            return

        try:
            self.stdscr.clear()

            # Get current state
            state = self.node.get_state_snapshot()

            # Draw sections
            y = 0
            y = self.draw_header(y)
            y = self.draw_status(y, state)
            y = self.draw_commands(y)

            # Show pose selection if waiting for number
            if self.waiting_for_number:
                y = self.draw_pose_selection(y)

            self.draw_log(y, state)

            self.stdscr.refresh()
        except curses.error:
            # Gracefully handle any drawing errors (e.g., during resize)
            pass

    def handle_input(self, key):
        """Handle keyboard input with Emacs-style combo keys."""
        # Handle terminal resize
        if key == curses.KEY_RESIZE:
            # Terminal was resized, update_display will handle size check
            return

        # ESC key - cancel combo
        if key == 27:  # ESC
            if self.waiting_for_number:
                self.waiting_for_number = False
                self.command_buffer = ""
                self.current_command = None
                self.node.log("Command cancelled")
            return

        # If waiting for number after I or R
        if self.waiting_for_number:
            if ord('1') <= key <= ord('9'):
                pose_idx = key - ord('1')  # 0-indexed

                # Execute the command with selected pose
                if self.current_command == 'I':
                    # Initialize with selected pose
                    if pose_idx < len(self.node.poses_data["initial_poses"]):
                        Thread(
                            target=self.node.initialize_localization,
                            args=(pose_idx,),
                            daemon=True
                        ).start()
                    else:
                        self.node.log(f"ERROR: Invalid pose number {pose_idx + 1}")

                elif self.current_command == 'R':
                    # Set route with selected goal
                    if pose_idx < len(self.node.poses_data["goal_poses"]):
                        Thread(
                            target=self.node.set_route,
                            args=(pose_idx,),
                            daemon=True
                        ).start()
                    else:
                        self.node.log(f"ERROR: Invalid goal number {pose_idx + 1}")

                # Reset combo state
                self.waiting_for_number = False
                self.command_buffer = ""
                self.current_command = None

            return

        # Normal command processing
        if key == ord('q') or key == ord('Q'):
            self.running = False

        elif key == ord('i') or key == ord('I'):
            # Start combo for Initialize pose
            self.waiting_for_number = True
            self.current_command = 'I'
            self.command_buffer = "I"
            self.node.log("Select initial pose (1-9)...")

        elif key == ord('r') or key == ord('R'):
            # Start combo for Route setting
            self.waiting_for_number = True
            self.current_command = 'R'
            self.command_buffer = "R"
            self.node.log("Select goal pose (1-9)...")

        elif key == ord('c') or key == ord('C'):
            # Clear route (immediate action, no combo)
            Thread(target=self.node.clear_route, daemon=True).start()

        elif key == ord('a') or key == ord('A'):
            # Autonomous mode (immediate action, no combo)
            Thread(target=self.node.set_autonomous_mode, daemon=True).start()

        elif key == ord('m') or key == ord('M'):
            # Manual mode (immediate action, no combo)
            Thread(target=self.node.set_manual_mode, daemon=True).start()

    def run(self):
        """Main TUI loop."""
        # Start ROS spinning thread
        def ros_spin():
            while self.running and rclpy.ok():
                self.node.spin_once_nonblocking()
                time.sleep(Config.STATUS_UPDATE_RATE)

        spin_thread = Thread(target=ros_spin, daemon=True)
        spin_thread.start()

        # Main display loop
        while self.running and rclpy.ok():
            try:
                # Update display
                self.update_display()

                # Handle input
                key = self.stdscr.getch()
                if key != -1:
                    self.handle_input(key)

                time.sleep(0.1)

            except KeyboardInterrupt:
                self.running = False
                break

        spin_thread.join(timeout=1.0)


def main():
    """Main entry point."""
    # Load poses data
    poses_data = load_poses()

    # Use first pose as default (can be changed via combo keys in TUI)
    init_idx = 0
    goal_idx = 0

    print(f"\nDefault poses:")
    print(f"  Initial: {poses_data['initial_poses'][init_idx]['name']}")
    print(f"  Goal:    {poses_data['goal_poses'][goal_idx]['name']}")
    print(f"\nAvailable: {len(poses_data['initial_poses'])} initial, {len(poses_data['goal_poses'])} goal poses")

    # Initialize ROS 2
    rclpy.init()

    # Create node with full poses data and default indices
    node = AutowareDriveNode(poses_data)
    node.current_initial_idx = init_idx
    node.current_goal_idx = goal_idx

    try:
        # Give node time to connect
        print("\nConnecting to Autoware...")
        for _ in range(10):
            node.spin_once_nonblocking()
            time.sleep(0.1)

        # Run TUI (uses Emacs-style combo keys for pose selection)
        print("Starting TUI...")
        print("Use [I]+[1-9] to select initial pose, [R]+[1-9] to select goal")
        print()
        curses.wrapper(lambda stdscr: AutowareTUI(stdscr, node).run())

    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return False

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
