#!/usr/bin/env bash
#
# scripts/launch_iris_mac.sh
# Launch Webots Iris example world on macOS

# move to project root (assumes this script lives in ./scripts)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# path to the built‑in iris.wbt
WORLD_FILE="$PROJECT_ROOT/worlds/iris.wbt"

# 1) If you just want to “open” the .app (no piping):
#    This will launch Webots in real‑time mode, but you won’t see its
#    stdout in this terminal.
open -a Webots --args \
     --mode=realtime \
     --stdout \
     --stderr \
     "$WORLD_FILE"

# 2) If you need to pipe logs (e.g. grep for RSSI), call the embedded CLI:
#    Adjust WEBOTS_APP if you installed Webots somewhere else.
WEBOTS_APP_PATH="/Applications/Webots.app/Contents/MacOS/webots"
if [ -x "$WEBOTS_APP_PATH" ]; then
  "$WEBOTS_APP_PATH" --mode=realtime \
                    --stdout \
                    --stderr \
                    "$WORLD_FILE" 2>&1 \
    | grep --line-buffered RSSI
else
  echo "❗ Could not find Webots CLI at $WEBOTS_APP_PATH"
  echo "  If you installed elsewhere, adjust WEBOTS_APP_PATH in this script."
  exit 1
fi
