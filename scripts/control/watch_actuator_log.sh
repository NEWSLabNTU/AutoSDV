#!/bin/bash
# Watch the latest actuator log from play_log directory

cd "$(dirname "$0")/../.."

# Record existing log directories before the new launch
echo "Waiting for new play_log directory to be created..."
if [ -d play_log ]; then
    EXISTING_LOGS=$(ls -t play_log 2>/dev/null | head -1)
    echo "Existing latest log: $EXISTING_LOGS"
else
    EXISTING_LOGS=""
fi

# Wait for a NEW log directory to appear
while true; do
    if [ -d play_log ]; then
        LATEST_LOG=$(ls -t play_log 2>/dev/null | head -1)
        # Check if this is a new directory (different from existing)
        if [ -n "$LATEST_LOG" ] && [ "$LATEST_LOG" != "$EXISTING_LOGS" ]; then
            echo "New log directory detected: $LATEST_LOG"
            break
        fi
    fi
    sleep 2
done

echo "Watching actuator logs in play_log/$LATEST_LOG/node/actuator_node/"

# Wait for actuator_node directory and err file to be created
while [ ! -f "play_log/$LATEST_LOG/node/actuator_node/err" ]; do
    echo "Waiting for actuator log file..."
    sleep 2
done

# Tail the actuator log (err file contains the actual logs)
tail -f "play_log/$LATEST_LOG/node/actuator_node/err"
