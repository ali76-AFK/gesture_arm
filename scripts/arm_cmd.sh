#!/usr/bin/env bash
# usage: scripts/arm_cmd.sh home|ready|inspect|open|close|stop
# Publishes once, after arm_control is matched (see arm_cmd.py). Needs the sim running.
cd "$(dirname "$0")/.."
./scripts/run.sh exec "python3 /ws/scripts/arm_cmd.py '$1'"
