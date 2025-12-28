#!/usr/bin/env python3
"""
Interactive tool to capture and save initial and goal poses from RViz.
Supports saving multiple poses for later selection in the TUI.

Usage:
1. Start Autoware with RViz
2. Run this script
3. Use RViz "2D Pose Estimate" to set initial pose → Press Enter
4. Use RViz "2D Goal Pose" to set goal pose → Press Enter
5. Repeat to add more poses
6. Type 'done' when finished

Poses are saved to poses.json in the same directory.
"""

import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class PoseRecorder(Node):
    """ROS 2 node to record poses from RViz."""

    def __init__(self):
        super().__init__("pose_recorder")

        self.latest_initial_pose = None
        self.latest_goal_pose = None

        # Subscribe to RViz pose topics
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            10,
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            10,
        )

        self.get_logger().info("Pose recorder initialized")
        self.get_logger().info("Listening for poses from RViz...")

    def initial_pose_callback(self, msg):
        """Store the latest initial pose."""
        self.latest_initial_pose = {
            "position": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
            },
            "orientation": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w,
            },
        }
        self.get_logger().info(
            f"Received initial pose: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})"
        )

    def goal_pose_callback(self, msg):
        """Store the latest goal pose."""
        self.latest_goal_pose = {
            "position": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
            },
            "orientation": {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w,
            },
        }
        self.get_logger().info(
            f"Received goal pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )


def load_existing_poses(poses_path):
    """Load existing poses file or return empty structure."""
    if poses_path.exists():
        try:
            with open(poses_path, "r") as f:
                data = json.load(f)
                # Handle old format (convert to new format)
                if "initial_pose" in data and "goal_pose" in data:
                    print("Converting old format to new format...")
                    return {
                        "initial_poses": [
                            {
                                "name": "Start 1",
                                "pose": data["initial_pose"]
                            }
                        ],
                        "goal_poses": [
                            {
                                "name": "Goal 1",
                                "pose": data["goal_pose"]
                            }
                        ]
                    }
                # Already new format
                if "initial_poses" in data and "goal_poses" in data:
                    return data
        except Exception as e:
            print(f"Warning: Could not load existing poses: {e}")

    # Return empty structure
    return {
        "initial_poses": [],
        "goal_poses": []
    }


def save_poses(poses_path, poses_data):
    """Save poses to JSON file."""
    with open(poses_path, "w") as f:
        json.dump(poses_data, f, indent=2)
    print(f"\n✓ Saved to {poses_path}")


def main():
    print("=" * 60)
    print("POSE RECORDER FOR AUTONOMOUS DRIVING")
    print("=" * 60)
    print()
    print("This tool captures poses from RViz and saves them to poses.json")
    print()
    print("Instructions:")
    print("  1. Use RViz '2D Pose Estimate' button to set initial pose")
    print("  2. Press Enter to record it")
    print("  3. Use RViz '2D Goal Pose' button to set goal pose")
    print("  4. Press Enter to record it")
    print("  5. Repeat to add more pose pairs")
    print("  6. Type 'done' when finished")
    print()

    # Initialize ROS 2
    rclpy.init()
    node = PoseRecorder()

    # Load existing poses
    poses_path = Path(__file__).parent / "poses.json"
    poses_data = load_existing_poses(poses_path)

    print(f"Loaded {len(poses_data['initial_poses'])} initial poses and {len(poses_data['goal_poses'])} goal poses")
    print()

    try:
        while True:
            # Get initial pose
            print("─" * 60)
            print("Step 1: Set INITIAL POSE in RViz (2D Pose Estimate)")
            print("        Then press Enter (or 'done' to finish)")
            print("─" * 60)

            response = input("> ").strip().lower()
            if response == "done":
                break

            # Spin to receive pose
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)

            if node.latest_initial_pose is None:
                print("⚠ No initial pose received. Please try again.")
                continue

            # Ask for name
            default_name = f"Start {len(poses_data['initial_poses']) + 1}"
            pose_name = input(f"Enter name for this initial pose [{default_name}]: ").strip()
            if not pose_name:
                pose_name = default_name

            initial_pose_entry = {
                "name": pose_name,
                "pose": node.latest_initial_pose
            }

            # Get goal pose
            print()
            print("─" * 60)
            print("Step 2: Set GOAL POSE in RViz (2D Goal Pose)")
            print("        Then press Enter")
            print("─" * 60)

            input("> ")

            # Spin to receive pose
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)

            if node.latest_goal_pose is None:
                print("⚠ No goal pose received. Please try again.")
                continue

            # Ask for name
            default_name = f"Goal {len(poses_data['goal_poses']) + 1}"
            pose_name = input(f"Enter name for this goal pose [{default_name}]: ").strip()
            if not pose_name:
                pose_name = default_name

            goal_pose_entry = {
                "name": pose_name,
                "pose": node.latest_goal_pose
            }

            # Add to collection
            poses_data['initial_poses'].append(initial_pose_entry)
            poses_data['goal_poses'].append(goal_pose_entry)

            print()
            print(f"✓ Added pose pair:")
            print(f"  Initial: {initial_pose_entry['name']}")
            print(f"  Goal:    {goal_pose_entry['name']}")

            # Reset for next iteration
            node.latest_initial_pose = None
            node.latest_goal_pose = None

            print()
            print("Add another pose pair? (Press Enter to continue, or type 'done')")
            response = input("> ").strip().lower()
            if response == "done":
                break

        # Save poses
        if poses_data['initial_poses'] and poses_data['goal_poses']:
            save_poses(poses_path, poses_data)
            print()
            print(f"Total saved: {len(poses_data['initial_poses'])} initial poses, {len(poses_data['goal_poses'])} goal poses")
        else:
            print("\n⚠ No poses captured. File not modified.")

    except KeyboardInterrupt:
        print("\n\nRecording cancelled")
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
