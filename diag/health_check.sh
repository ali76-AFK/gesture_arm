source /opt/ros/jazzy/setup.bash
echo "=== Gazebo world statistics, two samples 3 s apart (look at sim_time, paused, iterations) ==="
for i in 1 2; do
  gz topic -e -t /world/pick_place/stats -n 1 2>&1 | head -n 16
  echo "---"
  sleep 3
done
echo "=== ROS /clock ==="
r=$(timeout 6 ros2 topic hz /clock 2>&1 | grep "average rate" | tail -n 1); echo "${r:-NO /clock MESSAGES}"
echo "=== topic rates (5 s each) ==="
for t in /camera/image /camera/depth_image /detections /detections/image/compressed /dashboard/webcam/compressed /hand_state /joint_states; do
  r=$(timeout 5 ros2 topic hz $t 2>&1 | grep "average rate" | tail -n 1)
  printf "%-36s %s\n" "$t" "${r:-NO MESSAGES}"
done
echo "=== nodes alive ==="
ros2 node list 2>/dev/null | sort -u | grep -E "arm_control|object_detector|pick_place|gesture_interpreter|/move_group$|controller_manager|rosbridge"
echo "=== MoveIt scene: collision objects ==="
bash /ws/scripts/scene_check.sh
echo "=== MoveIt scene: allowed collision matrix ==="
python3 /ws/diag/acm_check.py
