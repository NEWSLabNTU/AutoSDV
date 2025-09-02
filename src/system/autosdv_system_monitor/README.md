# AutoSDV System Monitor

A ROS2 package that provides real-time monitoring of system topics through a web interface.

## Features

- Real-time monitoring of sensor and vehicle topics
- Web-based dashboard accessible at `http://localhost:8080`
- Configurable topic monitoring through YAML configuration
- Support for various message types:
  - Sensor data (LiDAR, Camera, IMU, GPS)
  - Vehicle status (velocity, control)
  - Diagnostics
  - System logs

## Usage

### Launch the monitor
```bash
ros2 launch autosdv_system_monitor autosdv_system_monitor.launch.yaml
```

### Configuration

Topics to monitor are configured in `config/monitor_topics.yaml`. Each topic entry contains:
- Message type
- Topic name
- Display name
- Parameter name (for enabling/disabling)
- QoS reliability setting

### Parameters

- `monitor_lidar`: Enable/disable LiDAR topic monitoring (default: true)
- `monitor_camera`: Enable/disable camera topic monitoring (default: true)
- `monitor_imu`: Enable/disable IMU topic monitoring (default: true)
- `monitor_gps`: Enable/disable GPS topic monitoring (default: true)
- `monitor_vehicle`: Enable/disable vehicle topic monitoring (default: true)
- `monitor_diagnostics`: Enable/disable diagnostics monitoring (default: true)
- `report_interval_sec`: Status update interval in seconds (default: 5.0)
- `web_server_host`: Web server host address (default: localhost)
- `web_server_port`: Web server port (default: 8080)
- `topics_config_file`: Path to topics configuration file

## Web Interface

Access the monitoring dashboard at `http://localhost:8080` (or the configured host/port).

The interface displays:
- Topic message rates and frequencies
- Time since last message
- Topic status (OK, STALE, ERROR, NO DATA)
- Detailed message information for specific topic types
- System diagnostics

## Dependencies

- ROS2 Humble
- Python packages: flask, watchdog, pyyaml
- Autoware messages: autoware_vehicle_msgs, autoware_control_msgs