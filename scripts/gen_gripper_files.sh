#!/usr/bin/env bash
# Generates the robot description and controllers file from the installed UR files (runs inside the container)
set -e
S=/opt/ros/jazzy/share/ur_simulation_gz
OUT=/ws/src/gesture_arm_sim
sed 's#</robot>#  <xacro:include filename="$(find gesture_arm_sim)/urdf/gripper.xacro"/>\n  <xacro:gripper parent="tool0"/>\n</robot>#' \
  $S/urdf/ur_gz.urdf.xacro > $OUT/urdf/ur_gripper.urdf.xacro
python3 - << 'PY'
import yaml
d = yaml.safe_load(open("/opt/ros/jazzy/share/ur_simulation_gz/config/ur_controllers.yaml"))
d["controller_manager"]["ros__parameters"]["gripper_controller"] = {
    "type": "joint_trajectory_controller/JointTrajectoryController"}
j = ["left_finger_joint", "right_finger_joint"]
d["gripper_controller"] = {"ros__parameters": {
    "joints": j, "command_interfaces": ["position"], "state_interfaces": ["position", "velocity"],
    "state_publish_rate": 50.0, "action_monitor_rate": 20.0, "allow_partial_joints_goal": False,
    "constraints": {"stopped_velocity_tolerance": 0.2, "goal_time": 0.0,
                    **{x: {"trajectory": 0.05, "goal": 0.05} for x in j}}}}
yaml.safe_dump(d, open("/ws/src/gesture_arm_sim/config/controllers.yaml", "w"), sort_keys=False)
print("wrote ur_gripper.urdf.xacro and controllers.yaml")
PY
