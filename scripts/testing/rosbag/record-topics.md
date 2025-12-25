# Record Topics Script

The `record-topics.sh` script allows you to record ROS2 topics listed in a text file.

## Usage

```bash
./record-topics.sh <topics_file> [optional_output_name]
```

### Arguments

- `<topics_file>`: Path to a text file containing the list of topics to record (one topic per line)
- `[optional_output_name]`: Optional name for the output bag directory. If not provided, a timestamped directory will be created.

## Topics File Format

The topics file should contain one topic per line. Empty lines and lines starting with `#` (comments) are ignored.

Example topics file:
```
# Vehicle topics  
/vehicle/status/velocity_status
/control/command/control_cmd

# Sensor topics
/sensing/lidar/velodyne_points
/sensing/imu/imu_data

# Camera topics
/sensing/camera/traffic_light/camera/image_raw/compressed
```

## Examples

1. Record topics from a file with automatic output naming:
   ```bash
   ./record-topics.sh sensing-topics.txt
   ```
   This will create a bag directory like `bag-20240116-143052`

2. Record topics with a custom output name:
   ```bash
   ./record-topics.sh sensing-topics.txt my-test-recording
   ```
   This will create a bag directory called `my-test-recording`

## Notes

- The script will validate that the topics file exists and contains at least one valid topic
- Topics with leading/trailing whitespace will be automatically trimmed
- The script will show all topics to be recorded before starting the recording
- Press Ctrl+C to stop the recording
- The recorded bag will be saved in the specified output directory
