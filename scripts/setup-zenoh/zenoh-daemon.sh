#!/usr/bin/env bash

DAEMON_NAME="rmw_zenohd"
DAEMON_CMD="ros2 run rmw_zenoh_cpp rmw_zenohd"
PID_FILE="/tmp/rmw_zenohd.pid"

get_daemon_pid() {
    # 使用 ps 命令找到真正的 daemon 進程
    ps aux | grep "$DAEMON_CMD" | grep -v grep | awk '{print $2}'
}

is_daemon_running() {
    local pid=$(get_daemon_pid)
    [ -n "$pid" ]
}

start_daemon() {
    if is_daemon_running; then
        echo "Daemon is already running"
        return 1
    fi
    
    echo "Starting RMW Zenoh daemon..."
    $DAEMON_CMD > /dev/null 2>&1 &
    local pid=$!
    echo $pid > "$PID_FILE"
    echo "Daemon started with PID: $pid"
}

stop_daemon() {
    local running_pid=$(get_daemon_pid)
    
    if [ -n "$running_pid" ]; then
        echo "Stopping daemon (PID: $running_pid)..."
        kill "$running_pid"
        rm -f "$PID_FILE"
        echo "Daemon stopped"
    else
        echo "Daemon is not running"
        # 清理可能存在的 stale PID file
        [ -f "$PID_FILE" ] && rm -f "$PID_FILE"
    fi
}

status_daemon() {
    local pid=$(get_daemon_pid)
    if [ -n "$pid" ]; then
        echo "Daemon is running (PID: $pid)"
        return 0
    else
        echo "Daemon is not running"
        return 1
    fi
}

case "$1" in
    start)
        start_daemon
        ;;
    stop)
        stop_daemon
        ;;
    restart)
        stop_daemon
        sleep 2
        start_daemon
        ;;
    status)
        status_daemon
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
