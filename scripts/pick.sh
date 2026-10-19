#!/usr/bin/env bash
# usage: scripts/pick.sh red_cube|green_cube|blue_cylinder|yellow_block
cd "$(dirname "$0")/.."
./scripts/run.sh exec "ros2 topic pub --once /pick_place_command std_msgs/msg/String \"{data: '$1'}\""
