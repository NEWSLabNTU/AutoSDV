#!/usr/bin/env python3
"""
Migrate rosbag from Autoware.Auto message types to Autoware 1.5.0 format.

This script converts deprecated autoware_auto_* message types to their
Autoware Universe equivalents (autoware_*_msgs).

Local adapted copy (AutoSDV Phase 3b Task 1)
---------------------------------------------
This is a local copy of
scripts/leodrive-bus-launch/scripts/migrate-to-autoware15.py (git submodule),
adapted here instead of editing the submodule directly. Differences from the
upstream script, needed to migrate data/sample-rosbag-replay/sample-rosbag:

1. Added an explicit mapping for the malformed connection type name
   "autoware_auto_vehicle_msgs/SteeringReport" (missing the "/msg/"
   segment) found on /vehicle/status/steering_status in the Autoware
   sample rosbag. The upstream pattern-based fallback only replaces
   "autoware_auto_" -> "autoware_" and therefore reproduces the same
   malformed shape ("autoware_vehicle_msgs/SteeringReport"), which the
   typestore cannot resolve.
2. Added /opt/ros/humble/share/ublox_msgs/msg to CUSTOM_MSG_PATHS so the
   ublox_msgs/msg/NavPVT type used by /sensing/gnss/ublox/navpvt can be
   registered and copied through unmodified (this topic is not migrated,
   only copied, but rosbags needs the type definition to do so).

Usage:
    ./migrate-to-autoware15.py <input_bag> <output_bag>

Example:
    ./migrate-to-autoware15.py all-sensors-bag1_compressed migrated_bag
"""

import argparse
import sys
from pathlib import Path

# Message type mappings: old -> new
# The message structure is typically identical, only package name changed
MESSAGE_TYPE_MAPPINGS = {
    # Vehicle messages
    "autoware_auto_vehicle_msgs/msg/VelocityReport": "autoware_vehicle_msgs/msg/VelocityReport",
    "autoware_auto_vehicle_msgs/msg/SteeringReport": "autoware_vehicle_msgs/msg/SteeringReport",
    # Malformed variant (missing "/msg/") seen in the Autoware sample rosbag
    "autoware_auto_vehicle_msgs/SteeringReport": "autoware_vehicle_msgs/msg/SteeringReport",
    "autoware_auto_vehicle_msgs/msg/GearReport": "autoware_vehicle_msgs/msg/GearReport",
    "autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport": "autoware_vehicle_msgs/msg/TurnIndicatorsReport",
    "autoware_auto_vehicle_msgs/msg/HazardLightsReport": "autoware_vehicle_msgs/msg/HazardLightsReport",
    "autoware_auto_vehicle_msgs/msg/ControlModeReport": "autoware_vehicle_msgs/msg/ControlModeReport",
    # Perception messages
    "autoware_auto_perception_msgs/msg/DetectedObjects": "autoware_perception_msgs/msg/DetectedObjects",
    "autoware_auto_perception_msgs/msg/TrackedObjects": "autoware_perception_msgs/msg/TrackedObjects",
    "autoware_auto_perception_msgs/msg/PredictedObjects": "autoware_perception_msgs/msg/PredictedObjects",
    # Planning messages
    "autoware_auto_planning_msgs/msg/Trajectory": "autoware_planning_msgs/msg/Trajectory",
    "autoware_auto_planning_msgs/msg/Path": "autoware_planning_msgs/msg/Path",
    # Control messages
    "autoware_auto_control_msgs/msg/AckermannControlCommand": "autoware_control_msgs/msg/Control",
    # Mapping messages
    "autoware_auto_mapping_msgs/msg/HADMapBin": "autoware_map_msgs/msg/LaneletMapBin",
}

# Topics to skip (not needed for Autoware 1.5.0 or handled differently)
SKIP_TOPICS: set[str] = set()

# Paths to search for custom message definitions
SCRIPT_DIR = Path(__file__).parent.resolve()
REPO_ROOT = SCRIPT_DIR.parent  # leodrive-bus-ood-launch/
CUSTOM_MSG_PATHS = [
    # velodyne_msgs (ros-humble-velodyne-msgs)
    Path("/opt/ros/humble/share/velodyne_msgs/msg"),
    # applanix_msgs (from submodule in src/, only present in leodrive-bus-launch checkouts)
    REPO_ROOT / "src/applanix/applanix_msgs/msg",
    # autoware_vehicle_msgs (Autoware 1.5.0)
    Path("/opt/autoware/1.5.0/share/autoware_vehicle_msgs/msg"),
    # ublox_msgs (ros-humble-ublox-msgs) - needed to copy through
    # /sensing/gnss/ublox/navpvt (ublox_msgs/msg/NavPVT) unmodified
    Path("/opt/ros/humble/share/ublox_msgs/msg"),
]


def get_new_message_type(old_type: str) -> str | None:
    """
    Get the Autoware 1.5.0 message type for a given Autoware.Auto type.
    Returns None if no mapping exists (keep original or skip).
    """
    # Direct mapping
    if old_type in MESSAGE_TYPE_MAPPINGS:
        return MESSAGE_TYPE_MAPPINGS[old_type]

    # Pattern-based mapping for unmapped autoware_auto_* types
    if old_type.startswith("autoware_auto_"):
        # Try to infer new name by removing "auto_"
        # autoware_auto_vehicle_msgs -> autoware_vehicle_msgs
        new_type = old_type.replace("autoware_auto_", "autoware_")
        print(f"  Warning: Inferred mapping {old_type} -> {new_type}", file=sys.stderr)
        return new_type

    # Not an autoware_auto message, keep as-is
    return None


def register_custom_messages(typestore):
    """
    Register custom message types from .msg files into the typestore.
    """
    from rosbags.typesys import get_types_from_msg

    registered = []

    for msg_dir in CUSTOM_MSG_PATHS:
        if not msg_dir.exists():
            print(f"  Note: Message path not found: {msg_dir}")
            continue

        # Infer package name from path
        # e.g., /opt/ros/humble/share/velodyne_msgs/msg -> velodyne_msgs
        pkg_name = msg_dir.parent.name

        for msg_file in msg_dir.glob("*.msg"):
            msg_name = msg_file.stem
            msg_type = f"{pkg_name}/msg/{msg_name}"

            # Skip if already in typestore
            if msg_type in typestore.types:
                continue

            try:
                msg_text = msg_file.read_text()
                types = get_types_from_msg(msg_text, msg_type)
                typestore.register(types)
                registered.append(msg_type)
            except Exception as e:
                print(f"  Warning: Failed to register {msg_type}: {e}")

    if registered:
        print(f"Registered {len(registered)} custom message types:")
        for t in registered:
            print(f"  - {t}")
        print()

    return registered


def migrate_rosbag(input_path: Path, output_path: Path, dry_run: bool = False):
    """
    Migrate a rosbag to Autoware 1.5.0 format.
    """
    try:
        from rosbags.rosbag2 import Reader, Writer
        from rosbags.typesys import Stores, get_typestore
        from rosbags.typesys.base import TypesysError
    except ImportError:
        print("Error: rosbags package not installed.", file=sys.stderr)
        print("Install with: pip install rosbags", file=sys.stderr)
        sys.exit(1)

    # Get ROS 2 Humble typestore for standard message definitions
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Register custom message types
    print("Loading custom message definitions...")
    register_custom_messages(typestore)

    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")
    print()

    # Analyze input bag
    with Reader(input_path) as reader:
        print("Topics in input bag:")
        print("-" * 60)

        topics_to_migrate = []
        topics_to_copy = []
        topics_to_skip = []

        for connection in reader.connections:
            topic = connection.topic
            msgtype = connection.msgtype

            if topic in SKIP_TOPICS:
                topics_to_skip.append((topic, msgtype, "configured skip"))
                continue

            new_msgtype = get_new_message_type(msgtype)
            if new_msgtype:
                topics_to_migrate.append((topic, msgtype, new_msgtype))
                print(f"  [MIGRATE] {topic}")
                print(f"            {msgtype} -> {new_msgtype}")
            else:
                topics_to_copy.append((topic, msgtype))
                print(f"  [COPY]    {topic}: {msgtype}")

        for topic, msgtype, reason in topics_to_skip:
            print(f"  [SKIP]    {topic}: {reason}")

        print()
        print(f"Summary: {len(topics_to_migrate)} to migrate, {len(topics_to_copy)} to copy, {len(topics_to_skip)} to skip")

    if dry_run:
        print("\nDry run complete. No output written.")
        return

    if not topics_to_migrate and not topics_to_copy:
        print("\nNo topics to process.")
        return

    # Perform migration
    print("\nMigrating...")

    with Reader(input_path) as reader:
        with Writer(output_path, version=8) as writer:
            # Create connection map
            conn_map = {}
            failed_topics = []

            for connection in reader.connections:
                topic = connection.topic
                msgtype = connection.msgtype

                if topic in SKIP_TOPICS:
                    continue

                new_msgtype = get_new_message_type(msgtype)
                out_msgtype = new_msgtype if new_msgtype else msgtype

                # Register connection in writer using typestore
                try:
                    conn_map[connection.id] = writer.add_connection(
                        topic,
                        out_msgtype,
                        typestore=typestore,
                        serialization_format=connection.ext.serialization_format,
                        offered_qos_profiles=connection.ext.offered_qos_profiles,
                    )
                except (KeyError, TypesysError) as e:
                    failed_topics.append((topic, out_msgtype, str(e)))
                    print(f"  Warning: Failed to add {topic} ({out_msgtype}): {e}")

            if failed_topics:
                print(f"\n  {len(failed_topics)} topics could not be registered")

            # Copy/migrate messages
            msg_count = 0
            for connection, timestamp, rawdata in reader.messages():
                if connection.id not in conn_map:
                    continue

                out_conn = conn_map[connection.id]

                # Copy raw CDR data (works when message structure is identical)
                writer.write(out_conn, timestamp, rawdata)
                msg_count += 1

                if msg_count % 10000 == 0:
                    print(f"  Processed {msg_count} messages...")

    print(f"\nMigration complete: {msg_count} messages written to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Migrate rosbag from Autoware.Auto to Autoware 1.5.0 format"
    )
    parser.add_argument("input_bag", type=Path, help="Input rosbag directory")
    parser.add_argument("output_bag", type=Path, help="Output rosbag directory")
    parser.add_argument(
        "--dry-run", "-n", action="store_true", help="Analyze only, don't write output"
    )

    args = parser.parse_args()

    if not args.input_bag.exists():
        print(f"Error: Input bag not found: {args.input_bag}", file=sys.stderr)
        sys.exit(1)

    if args.output_bag.exists() and not args.dry_run:
        print(f"Error: Output path already exists: {args.output_bag}", file=sys.stderr)
        print("Remove it first or choose a different output path.", file=sys.stderr)
        sys.exit(1)

    migrate_rosbag(args.input_bag, args.output_bag, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
