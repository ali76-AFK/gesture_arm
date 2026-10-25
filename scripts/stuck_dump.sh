source /opt/ros/jazzy/setup.bash
echo "=== processes in this container ==="; ps -eo pid,etime,pcpu,args | grep -E "move_group|arm_control|pick_place|object_detector|gesture_interpreter|gz sim" | grep -v grep | cut -c1-150
echo "=== nodes ==="; ros2 node list 2>/dev/null | sort -u | grep -E "move_group|arm_control|pick_place|gesture_interpreter|controller_manager|object_detector"
echo "=== controllers ==="; ros2 control list_controllers 2>&1 | head -n 8
echo "=== action servers ==="; ros2 action list -t 2>&1 | head -n 12
echo "=== collision objects in the planning scene ==="; ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 8}}" | grep -oE " id='[a-z_]+'" || echo "NONE"
echo "=== snapshot ==="; python3 /ws/scripts/stuck_dump.py
