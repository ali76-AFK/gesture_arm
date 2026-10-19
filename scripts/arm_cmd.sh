#!/usr/bin/env bash
# usage: scripts/arm_cmd.sh home|ready|inspect|stop
cd "$(dirname "$0")/.."
./scripts/run.sh exec "ros2 topic pub --once /arm_command std_msgs/msg/String \"{data: '$1'}\""
