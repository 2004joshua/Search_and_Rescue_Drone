#!/usr/bin/env bash
# builds & runs SITL from project root
cd "$(dirname "$0")/../ardupilot"
./waf configure --board sitl
./waf build
./build/sitl/bin/arducopter --model quad --serial0=udpclient:127.0.0.1:9002
./build/sitl/bin/arducopter \
    --model quad \
    --serial0=udpclient:127.0.0.1:9002 \      # Webots controller
    --serial1=udpclient:127.0.0.1:14550       # MAVProxy
