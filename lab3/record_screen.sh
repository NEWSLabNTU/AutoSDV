#!/bin/bash
# Step 3: record the screen while you drive the lap in the planning simulator; Ctrl-C to stop.
# Usage: lab3/record_screen.sh ../team3_psim.mp4   (an X11 desktop: the car's VNC session)
[ -n "$1" ] || { echo "Usage: $0 ../team3_psim.mp4" >&2; exit 2; }
size=$(xdpyinfo | awk '/dimensions:/ {print $2}')
ffmpeg -f x11grab -framerate 10 -video_size "$size" -i "$DISPLAY" \
    -c:v libx264 -preset veryfast -crf 30 -pix_fmt yuv420p "$1"
