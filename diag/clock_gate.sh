source /opt/ros/jazzy/setup.bash
for i in 1 2 3; do
  out=$(timeout 6 ros2 topic echo /clock --once 2>/dev/null | grep -m1 "sec:" | tr -d ' ')
  if [ -n "$out" ]; then echo "CLOCK OK ($out)"; exit 0; fi
  sleep 3
done
echo "CLOCK MISSING: do not start arm_control, object_detector or pick_place yet"
exit 1
