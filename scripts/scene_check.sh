source /opt/ros/jazzy/setup.bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 8}}" | grep -oE " id='[a-z_]+'" || echo "NO COLLISION OBJECTS FOUND"
