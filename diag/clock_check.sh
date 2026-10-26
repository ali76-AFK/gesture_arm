source /opt/ros/jazzy/setup.bash
echo "=== one /clock message (echo adapts its QoS to the publisher) ==="
timeout 6 ros2 topic echo /clock --once 2>&1 | head -n 6
echo "=== /clock publishers and their QoS ==="
ros2 topic info /clock -v 2>&1 | head -n 40
echo "=== use_sim_time on the nodes that follow the simulation clock ==="
for n in /arm_control /object_detector /move_group; do echo -n "$n: "; timeout 5 ros2 param get $n use_sim_time 2>&1 | head -n 1; done
echo "=== nodes (no daemon, 5 s discovery) ==="
ros2 node list --no-daemon --spin-time 5 2>&1 | sort -u | head -n 40
echo "=== processes in this container ==="
ps -eo pid,etime,pcpu,args | grep -E "parameter_bridge|gz sim|move_group|controller_manager|arm_control|object_detector|pick_place|rosbridge" | grep -v grep | cut -c1-175
