#!/usr/bin/env bash
#
# from project root:
# 1) tell Webots where to find your controllers
# 2) launch the main world with stdout/stderr visible

# move up to project root (in case you invoked this from scripts/)
cd "$(dirname "$0")/.."

# point Webots at your controllers folder
export WEBOTS_CONTROLLER_PATH="$PWD/controllers"

# launch Webots on your primary world
# --stdout/--stderr just pipes all logs into your terminal
webots --stdout --stderr worlds/my_world.wbt
