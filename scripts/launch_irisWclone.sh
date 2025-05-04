#!/usr/bin/env bash
#
# scripts/launch_iris.sh
# Launch Webots using the ArduPilot Iris example world

# move to project root (assumes this script lives in ./scripts)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# path to the built-in iris.wbt
WORLD_FILE="$PROJECT_ROOT/ardupilot/libraries/SITL/examples/Webots_Python/worlds/iris.wbt"

# optional: override WEBOTS_HOME if you installed Webots somewhere else
# export WEBOTS_HOME="/usr/local/webots"

# launch Webots in real-time mode, printing its stdout/stderr to your terminal
# webots --mode=realtime --stdout --stderr "$WORLD_FILE"
webots --mode=realtime --stdout --stderr "$WORLD_FILE" 2>&1 | grep --line-buffered RSSI
# webots --mode=realtime --stdout --stderr "$WORLD_FILE" 2>&1 | grep --line-buffered ardupilot_vehicle_controller