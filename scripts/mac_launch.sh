#!/usr/bin/env bash
#
# scripts/launch_iris_open.sh
# Launch Webots (via macOS `open`) using the ArduPilot Iris example world

# move to project root (assumes this script lives in ./scripts)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# path to the built-in iris.wbt
WORLD_FILE="$PROJECT_ROOT/ardupilot/libraries/SITL/examples/Webots_Python/worlds/iris.wbt"

# optional: override the Webots app bundle location if you installed it elsewhere
WEBOTS_APP="/Applications/Webots.app"

if [ ! -d "$WEBOTS_APP" ]; then
  echo "Error: Webots app not found at $WEBOTS_APP"
  exit 1
fi

# launch Webots via `open` so it picks up its Cocoa/Qt plugins,
# passing along all the usual command-line flags:
open -a "$WEBOTS_APP" --args \
     --mode=realtime \
     --stdout \
     --stderr \
     "$WORLD_FILE"

#./ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm