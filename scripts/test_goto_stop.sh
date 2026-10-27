#!/bin/bash
# test_goto_stop.sh: runs INSIDE the sim container (via run.sh exec). MOVES the arm.
# Needs: sim + arm_control + goto_point running, arm at home, 2 waypoints already set.
# Starts 'go', sends /arm_command stop 0.5 s after the first motion line, then reports where the arm ended up.
LOG=/tmp/goto_go.log
rm -f "$LOG"
tf() { timeout 6 ros2 run tf2_ros tf2_echo base_link gripper_tcp 2>&1 | grep -E "Translation" | head -n 1; }
echo "tf before:      $(tf)"
T0=$(date +%s.%N)
timeout 40 python3 -u /ws/scripts/goto_cli.py go > "$LOG" 2>&1 &
GO=$!
for i in $(seq 1 100); do
  grep -qE "moving:[12]" "$LOG" && break
  sleep 0.1
done
if ! grep -qE "moving:[12]" "$LOG"; then
  echo "no 'moving:' line within 10 s, stop NOT sent"
  kill $GO 2>/dev/null
  cat "$LOG"
  exit 1
fi
sleep 0.5
T1=$(date +%s.%N)
ros2 topic pub --once /arm_command std_msgs/msg/String "{data: stop}"
T2=$(date +%s.%N)
wait $GO
T3=$(date +%s.%N)
python3 -c 'import sys; a,b,c,d=map(float,sys.argv[1:]); print("stop publish began +%.2f s, ended +%.2f s, go CLI ended +%.2f s (all after go launch)" % (b-a,c-a,d-a))' "$T0" "$T1" "$T2" "$T3"
echo "--- go CLI output ---"
cat "$LOG"
echo "tf right after: $(tf)"
sleep 2
echo "tf 2 s later:   $(tf)"
