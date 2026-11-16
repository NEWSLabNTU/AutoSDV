#!/bin/bash
# Step 3: record the screen while you drive the lap in the planning simulator; Ctrl-C to stop.
# Usage: lab3/record_screen.sh lab3/psim.mp4
size=$(xdpyinfo | awk '/dimensions:/ {print $2}')
ffmpeg -f x11grab -framerate 10 -video_size "$size" -i "$DISPLAY" \
    -c:v libx264 -preset veryfast -crf 30 -pix_fmt yuv420p "$1"
