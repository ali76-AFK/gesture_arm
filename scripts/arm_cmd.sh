#!/usr/bin/env bash
# usage: scripts/arm_cmd.sh home|ready|inspect|open|close|stop
# Publishes once, after arm_control is matched (see arm_cmd.py).
# stop waits for arm_control, goto_point and pick_place, and is sent even if one of them never matches.
cd "$(dirname "$0")/.."
need=""
[ "$1" = "stop" ] && need="arm_control goto_point pick_place"
./scripts/run.sh exec "python3 /ws/scripts/arm_cmd.py '$1' $need"
