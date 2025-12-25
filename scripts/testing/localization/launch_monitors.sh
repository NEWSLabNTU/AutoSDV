#!/bin/bash
# Launch all localization monitoring tools in a tmux session

SESSION="outdoor-test"

# Check if session already exists
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "Session '$SESSION' already exists. Attaching..."
    tmux attach-session -t $SESSION
    exit 0
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../.." && pwd )"

echo "Creating tmux session '$SESSION' for outdoor localization testing..."

# Source ROS workspace
SOURCE_CMD="cd $WORKSPACE_DIR && source install/setup.bash"

# Create new session with GPS monitor
tmux new-session -d -s $SESSION -n "GPS" "$SOURCE_CMD && $SCRIPT_DIR/monitor_gps.py"

# Create window for localization monitor
tmux new-window -t $SESSION:1 -n "Localization" "$SOURCE_CMD && $SCRIPT_DIR/monitor_localization.py"

# Create window for map bounds checker
tmux new-window -t $SESSION:2 -n "Map Bounds" "$SOURCE_CMD && $SCRIPT_DIR/check_map_bounds.py"

# Create window for topic monitoring
tmux new-window -t $SESSION:3 -n "Topics" "$SOURCE_CMD && echo 'Topic Monitor - Use these commands:
  ros2 topic hz /sensing/gnss/ublox/nav_sat_fix
  ros2 topic echo /localization/kinematic_state
  ros2 topic echo /localization/pose_estimator/transform_probability
' && bash"

# Select first window and attach
tmux select-window -t $SESSION:0
tmux attach-session -t $SESSION

echo ""
echo "Tmux session started. Use:"
echo "  Ctrl+B then 0/1/2/3 to switch windows"
echo "  Ctrl+B then d to detach"
echo "  tmux attach -t $SESSION to re-attach"
