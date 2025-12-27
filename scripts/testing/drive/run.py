#!/usr/bin/env python3
"""
Autonomous driving script for Autoware using standard rclpy API.
Reads poses from poses.json and drives the vehicle autonomously to the goal.

Updated for modern Autoware (2024/2025):
- Uses /api/localization/initialize service
- Uses /api/routing/set_route_points service
- Uses /api/routing/clear_route service
- Uses /api/operation_mode/change_to_autonomous service

IMPORTANT: The initial and goal poses MUST be on connected lanes in the lanelet2 map.
If route planning fails, the poses may not have a valid path between them.
Use RViz's "2D Pose Estimate" and "2D Goal Pose" buttons to select valid poses,
or use get_carla_spawn_points.py to find valid spawn points from CARLA.
"""

import json
import math
import time
import sys
from pathlib import Path

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
    LOCALIZATION_TIMEOUT = 90.0
    POSITION_DATA_TIMEOUT = 30.0
    ROUTE_SET_TIMEOUT = 60.0
    AUTONOMOUS_MODE_TIMEOUT = 30.0
    DRIVING_TIMEOUT = 300.0

    # Retry settings
    MAX_RETRIES = 3
    RETRY_DELAY = 2.0

    # Goal tolerance
    GOAL_DISTANCE_THRESHOLD = 5.0

    # Stuck detection
    STUCK_VELOCITY_THRESHOLD = 0.1
    STUCK_COUNT_THRESHOLD = 5


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
        poses = json.load(f)

    print(f"  Loaded poses from {poses_path}")
    return poses


def calculate_distance(x1, y1, x2, y2):
    """Calculate 2D distance between two points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class AutowareDriveNode(Node):
    """ROS 2 node for autonomous driving with Autoware."""

    def __init__(self, poses):
        super().__init__("autoware_drive_node")

        self.poses = poses
        self.initial_pose = poses["initial_pose"]
        self.goal_pose = poses["goal_pose"]

        # Vehicle state
        self.current_position = None
        self.current_velocity = 0.0
        self.route_state = None
        self.operation_mode = None
        self.localization_state = None

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

        self.get_logger().info("Autoware Drive Node initialized")

    def velocity_callback(self, msg):
        """Update current velocity."""
        self.current_velocity = msg.longitudinal_velocity

    def odometry_callback(self, msg):
        """Update current position from odometry."""
        self.current_position = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
        }

    def route_state_callback(self, msg):
        """Update route state."""
        self.route_state = msg.state

    def operation_mode_callback(self, msg):
        """Update operation mode."""
        self.operation_mode = msg.mode

    def localization_state_callback(self, msg):
        """Update localization initialization state."""
        self.localization_state = msg.state

    def spin_and_update(self, duration_sec=0.1):
        """Spin to process callbacks."""
        rclpy.spin_once(self, timeout_sec=duration_sec)

    def call_service_with_retry(
        self, client, request, service_name, timeout_sec=None, max_retries=None
    ):
        """Call a service with retry logic."""
        if timeout_sec is None:
            timeout_sec = Config.SERVICE_TIMEOUT
        if max_retries is None:
            max_retries = Config.MAX_RETRIES

        for attempt in range(max_retries):
            # Wait for service
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().warning(
                    f"{service_name} service not available (attempt {attempt + 1}/{max_retries})"
                )
                if attempt < max_retries - 1:
                    time.sleep(Config.RETRY_DELAY)
                continue

            # Call service
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                result = future.result()
                if result.status.success:
                    return True, result
                else:
                    self.get_logger().warning(
                        f"{service_name} failed: {result.status.message} "
                        f"(attempt {attempt + 1}/{max_retries})"
                    )
            else:
                self.get_logger().warning(
                    f"{service_name} call timed out (attempt {attempt + 1}/{max_retries})"
                )

            if attempt < max_retries - 1:
                time.sleep(Config.RETRY_DELAY)

        return False, None

    def initialize_localization(self):
        """Initialize localization using the Autoware API service."""
        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = self.initial_pose["position"]["x"]
        pose_msg.pose.pose.position.y = self.initial_pose["position"]["y"]
        pose_msg.pose.pose.position.z = self.initial_pose["position"]["z"]

        pose_msg.pose.pose.orientation.x = self.initial_pose["orientation"]["x"]
        pose_msg.pose.pose.orientation.y = self.initial_pose["orientation"]["y"]
        pose_msg.pose.pose.orientation.z = self.initial_pose["orientation"]["z"]
        pose_msg.pose.pose.orientation.w = self.initial_pose["orientation"]["w"]

        # Set covariance (small values indicate high confidence)
        pose_msg.pose.covariance[0] = 0.25  # x variance
        pose_msg.pose.covariance[7] = 0.25  # y variance
        pose_msg.pose.covariance[35] = 0.06853891909122467  # yaw variance

        # Call service
        request = InitializeLocalization.Request()
        request.pose.append(pose_msg)

        success, _ = self.call_service_with_retry(
            self.init_localization_client,
            request,
            "Initialize localization",
        )
        return success

    def wait_for_localization_ready(self, timeout_sec=None):
        """Wait for localization to be initialized."""
        if timeout_sec is None:
            timeout_sec = Config.LOCALIZATION_TIMEOUT

        start_time = time.time()
        last_print_time = 0

        while (time.time() - start_time) < timeout_sec:
            self.spin_and_update(0.5)

            if self.localization_state == LocalizationStateEnum.INITIALIZED:
                return True

            elapsed = time.time() - start_time
            # Print status every 5 seconds
            if int(elapsed) >= last_print_time + 5:
                state_name = self._get_localization_state_name()
                print(f"     Waiting for localization... {elapsed:.0f}s ({state_name})")
                last_print_time = int(elapsed)

        return False

    def _get_localization_state_name(self):
        """Get human-readable localization state name."""
        state_names = {
            LocalizationStateEnum.UNKNOWN: "UNKNOWN",
            LocalizationStateEnum.UNINITIALIZED: "UNINITIALIZED",
            LocalizationStateEnum.INITIALIZING: "INITIALIZING",
            LocalizationStateEnum.INITIALIZED: "INITIALIZED",
        }
        return state_names.get(self.localization_state, f"STATE_{self.localization_state}")

    def wait_for_position_data(self, timeout_sec=None):
        """Wait until we receive position data from odometry."""
        if timeout_sec is None:
            timeout_sec = Config.POSITION_DATA_TIMEOUT

        start_time = time.time()

        while (time.time() - start_time) < timeout_sec:
            self.spin_and_update(0.5)

            if self.current_position is not None:
                return True

            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 0:
                print(f"     Waiting for position data... {elapsed:.0f}s")

        return False

    def clear_route(self):
        """Clear existing route."""
        request = ClearRoute.Request()
        success, _ = self.call_service_with_retry(
            self.clear_route_client,
            request,
            "Clear route",
        )
        return success

    def set_route(self):
        """Set route to goal pose."""
        request = SetRoutePoints.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "map"

        # Set goal pose
        goal = Pose()
        goal.position.x = self.goal_pose["position"]["x"]
        goal.position.y = self.goal_pose["position"]["y"]
        goal.position.z = self.goal_pose["position"]["z"]
        goal.orientation.x = self.goal_pose["orientation"]["x"]
        goal.orientation.y = self.goal_pose["orientation"]["y"]
        goal.orientation.z = self.goal_pose["orientation"]["z"]
        goal.orientation.w = self.goal_pose["orientation"]["w"]

        request.goal = goal
        request.option.allow_goal_modification = True

        success, _ = self.call_service_with_retry(
            self.set_route_client,
            request,
            "Set route",
            timeout_sec=Config.ROUTE_SET_TIMEOUT,
        )
        return success

    def wait_for_route_set(self, timeout_sec=None):
        """Wait for route to be set."""
        if timeout_sec is None:
            timeout_sec = Config.ROUTE_SET_TIMEOUT

        start_time = time.time()
        last_print_time = 0

        while (time.time() - start_time) < timeout_sec:
            self.spin_and_update(0.5)

            if self.route_state == RouteStateEnum.SET:
                return True

            elapsed = time.time() - start_time
            if int(elapsed) >= last_print_time + 5:
                state_name = self._get_route_state_name()
                print(f"     Waiting for route... {elapsed:.0f}s ({state_name})")
                last_print_time = int(elapsed)

        return False

    def _get_route_state_name(self):
        """Get human-readable route state name."""
        state_names = {
            RouteStateEnum.UNKNOWN: "UNKNOWN",
            RouteStateEnum.UNSET: "UNSET",
            RouteStateEnum.SET: "SET",
            RouteStateEnum.ARRIVED: "ARRIVED",
            RouteStateEnum.CHANGING: "CHANGING",
        }
        return state_names.get(self.route_state, f"STATE_{self.route_state}")

    def set_autonomous_mode(self):
        """Change to autonomous operation mode."""
        request = ChangeOperationMode.Request()
        success, _ = self.call_service_with_retry(
            self.change_mode_client,
            request,
            "Change to autonomous mode",
            timeout_sec=Config.AUTONOMOUS_MODE_TIMEOUT,
        )
        return success

    def monitor_progress(self):
        """Monitor vehicle progress until goal is reached."""
        goal_x = self.goal_pose["position"]["x"]
        goal_y = self.goal_pose["position"]["y"]

        if self.current_position:
            start_x = self.current_position["x"]
            start_y = self.current_position["y"]
            total_distance = calculate_distance(start_x, start_y, goal_x, goal_y)
        else:
            total_distance = None

        print("\n" + "=" * 60)
        print("MONITORING VEHICLE PROGRESS")
        print("=" * 60)

        start_time = time.time()
        last_update = start_time
        stuck_counter = 0

        try:
            while True:
                current_time = time.time()
                elapsed = current_time - start_time

                # Update every 2 seconds
                if current_time - last_update >= 2.0:
                    self.spin_and_update(0.1)

                    if self.current_position:
                        curr_x = self.current_position["x"]
                        curr_y = self.current_position["y"]
                        distance_to_goal = calculate_distance(
                            curr_x, curr_y, goal_x, goal_y
                        )

                        if total_distance and total_distance > 0:
                            progress = (
                                (total_distance - distance_to_goal) / total_distance
                            ) * 100
                            progress = max(0.0, min(100.0, progress))
                        else:
                            progress = 0.0

                        print(
                            f"[{elapsed:5.1f}s] Speed: {self.current_velocity:.2f} m/s | "
                            f"Distance: {distance_to_goal:.1f}m | Progress: {progress:.1f}%"
                        )

                        # Check if arrived via route state
                        if self.route_state == RouteStateEnum.ARRIVED:
                            print("=" * 60)
                            print("\n  ARRIVED at goal!")
                            break

                        # Check if close enough to goal
                        if distance_to_goal < Config.GOAL_DISTANCE_THRESHOLD:
                            print("=" * 60)
                            print(
                                f"\n  Vehicle reached goal! "
                                f"Final distance: {distance_to_goal:.2f} m"
                            )
                            break

                        # Check if stuck
                        if abs(self.current_velocity) < Config.STUCK_VELOCITY_THRESHOLD:
                            stuck_counter += 1
                            if stuck_counter > Config.STUCK_COUNT_THRESHOLD:
                                print(
                                    f"  Vehicle may be stuck "
                                    f"(speed < {Config.STUCK_VELOCITY_THRESHOLD} m/s)"
                                )
                        else:
                            stuck_counter = 0
                    else:
                        print(f"[{elapsed:5.1f}s] Waiting for position data...")

                    last_update = current_time

                # Timeout
                if elapsed > Config.DRIVING_TIMEOUT:
                    print(
                        f"\n  Timeout: Did not reach goal within "
                        f"{Config.DRIVING_TIMEOUT / 60:.0f} minutes"
                    )
                    break

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")


def main():
    print("=" * 60)
    print("AUTOWARE AUTONOMOUS DRIVING")
    print("=" * 60)

    # Load poses
    print("\n1. Loading poses...")
    poses = load_poses()

    initial_pose = poses["initial_pose"]
    goal_pose = poses["goal_pose"]

    total_distance = calculate_distance(
        initial_pose["position"]["x"],
        initial_pose["position"]["y"],
        goal_pose["position"]["x"],
        goal_pose["position"]["y"],
    )
    print(f"   Total distance to goal: {total_distance:.2f} meters")

    # Initialize ROS 2
    rclpy.init()

    # Create node
    node = AutowareDriveNode(poses)

    try:
        # Spin a few times to get initial state
        print("\n2. Connecting to Autoware...")
        for _ in range(20):
            node.spin_and_update(0.1)

        # Initialize localization
        print("\n3. Initializing localization...")
        print(
            f"   Initial pose: "
            f"({initial_pose['position']['x']:.2f}, "
            f"{initial_pose['position']['y']:.2f})"
        )

        if not node.initialize_localization():
            print("   Warning: Localization initialization request may have failed")

        # Wait for localization to converge
        print("   Waiting for localization to converge...")
        if not node.wait_for_localization_ready():
            print("   Warning: Localization may not be fully ready, continuing...")
        else:
            print("   Localization ready")

        # Wait for position data
        print("   Waiting for position data...")
        if not node.wait_for_position_data():
            print("   Warning: No position data received")
        else:
            pos = node.current_position
            print(f"   Position confirmed: ({pos['x']:.2f}, {pos['y']:.2f})")

        # Clear existing route
        print("\n4. Clearing existing route...")
        if not node.clear_route():
            print("   Warning: Failed to clear route, continuing...")
        else:
            print("   Route cleared")

        # Small delay after clearing
        time.sleep(1.0)

        # Set route
        print("\n5. Setting route to goal...")
        print(
            f"   Goal pose: "
            f"({goal_pose['position']['x']:.2f}, "
            f"{goal_pose['position']['y']:.2f})"
        )

        if not node.set_route():
            print("   Failed to set route")
            print("\nTroubleshooting:")
            print(
                "  - The initial and goal poses may not be connected in the lanelet2 map"
            )
            print(
                "  - Try using RViz to select poses with '2D Pose Estimate' and '2D Goal Pose'"
            )
            print("  - Check that localization has converged properly")
            print("  - Check Autoware logs for route planning errors")
            return False

        # Wait for route to be processed
        print("   Waiting for route to be ready...")
        if not node.wait_for_route_set():
            print("   Warning: Route may not be fully processed")
        else:
            print("   Route is set and ready")

        # Engage autonomous mode
        print("\n6. Engaging autonomous mode...")
        if not node.set_autonomous_mode():
            print("   Failed to engage autonomous mode")
            print("\nTroubleshooting:")
            print("  - Check that all Autoware modules are running")
            print("  - Verify there are no emergency stops active")
            print("  - Check operation mode state in RViz")
            return False

        print("   AUTONOMOUS MODE ENGAGED")
        print("   Vehicle is now driving autonomously!")

        # Brief pause before monitoring
        time.sleep(1.0)

        # Monitor progress
        node.monitor_progress()

        print("\n" + "=" * 60)
        print("AUTONOMOUS DRIVING COMPLETE")
        print("=" * 60)

        return True

    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user")
        return False

    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return False

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
