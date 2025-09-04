#!/usr/bin/env bash
#
# AutoSDV Launch Script with Process Group Management
# Ensures all child processes are properly cleaned up on exit
#

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

export RCUTILS_COLORIZED_OUTPUT=1  # Force colored output

# Make this script the leader of a new process group
set -m

# Function to recursively kill a process tree
kill_process_tree() {
    local pid=$1
    local signal=${2:-TERM}

    # Get all child processes
    local children=$(pgrep -P "$pid" 2>/dev/null)

    # Kill children first (depth-first)
    for child in $children; do
        kill_process_tree "$child" "$signal"
    done

    # Then kill the parent
    kill -"$signal" "$pid" 2>/dev/null || true
}

# Cleanup function
cleanup() {
    local exit_code=$?

    echo ""
    echo "═══════════════════════════════════════"
    echo "  Shutting down AutoSDV..."
    echo "═══════════════════════════════════════"

    # Get the process group ID (negative PID kills the entire group)
    local pgid=$(ps -o pgid= -p $$ | tr -d ' ')

    if [ -n "$ROS_LAUNCH_PID" ] && kill -0 "$ROS_LAUNCH_PID" 2>/dev/null; then
        echo "→ Stopping ROS2 launch (PID: $ROS_LAUNCH_PID)..."

        # First, try graceful shutdown with SIGINT (like Ctrl+C)
        kill -INT "$ROS_LAUNCH_PID" 2>/dev/null || true

        # Wait up to 5 seconds for graceful shutdown
        for i in {1..10}; do
            if ! kill -0 "$ROS_LAUNCH_PID" 2>/dev/null; then
                echo "→ ROS2 launch stopped gracefully"
                break
            fi
            sleep 0.5
        done

        # If still running, use SIGTERM
        if kill -0 "$ROS_LAUNCH_PID" 2>/dev/null; then
            echo "→ Sending SIGTERM to process tree..."
            kill_process_tree "$ROS_LAUNCH_PID" TERM
            sleep 2
        fi

        # Final cleanup with SIGKILL if needed
        if kill -0 "$ROS_LAUNCH_PID" 2>/dev/null; then
            echo "→ Force killing remaining processes..."
            kill_process_tree "$ROS_LAUNCH_PID" KILL
        fi
    fi

    # Clean up any orphaned ROS processes by name
    echo "→ Cleaning up any remaining ROS processes..."

    # Array of process patterns to clean up
    local process_patterns=(
        "ros2.*autosdv"
        "autosdv_vehicle_interface"
        "autosdv_launch"
        "component_container"
        "robot_state_publisher"
    )

    for pattern in "${process_patterns[@]}"; do
        if pgrep -f "$pattern" > /dev/null 2>&1; then
            pkill -TERM -f "$pattern" 2>/dev/null || true
        fi
    done

    # Brief wait then force kill any stubborn processes
    sleep 1
    for pattern in "${process_patterns[@]}"; do
        if pgrep -f "$pattern" > /dev/null 2>&1; then
            pkill -KILL -f "$pattern" 2>/dev/null || true
        fi
    done

    echo "═══════════════════════════════════════"
    echo "  AutoSDV shutdown complete"
    echo "═══════════════════════════════════════"

    exit $exit_code
}

# Set up comprehensive signal handling
trap cleanup EXIT
trap cleanup SIGINT
trap cleanup SIGTERM
trap cleanup SIGHUP

# Display startup message
echo "═══════════════════════════════════════"
echo "  Starting AutoSDV Launch System"
echo "═══════════════════════════════════════"
echo ""
echo "→ System Monitor: http://localhost:8080/"
echo "→ Press Ctrl+C to shutdown cleanly"
echo ""
echo "═══════════════════════════════════════"
echo ""

# Source ROS environment
source install/setup.bash

# Launch ROS2 in the foreground so we can properly manage it
# Using exec would replace this script, losing our cleanup handling
# So we run it as a child process but in the foreground
ros2 launch autosdv_launch autosdv.launch.yaml 2>&1 | tee log.txt &
ROS_LAUNCH_PID=$!

# Wait for the launch process
# This will be interrupted by signals, which trigger cleanup
wait $ROS_LAUNCH_PID
WAIT_RESULT=$?

# If we get here normally (not via signal), still run cleanup
if [ $WAIT_RESULT -ne 0 ]; then
    echo "ROS2 launch exited with code $WAIT_RESULT"
fi

# Cleanup will be called by the EXIT trap
