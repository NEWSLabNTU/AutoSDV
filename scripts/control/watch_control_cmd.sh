#!/bin/bash
# Watch /control/command/control_cmd topic with auto-restart

cd "$(dirname "$0")/../.."
source install/setup.bash

while true; do
    echo "Waiting for /control/command/control_cmd topic..."
    ros2 topic echo /control/command/control_cmd
    echo "Topic echo stopped. Restarting in 2 seconds..."
    sleep 2
done
