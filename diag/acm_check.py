#!/usr/bin/env python3
"""acm_check: does MoveIt's allowed-collision matrix contain the gripper links arm_control adds? (read-only)"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

rclpy.init()
n = Node("acm_check")
cli = n.create_client(GetPlanningScene, "/get_planning_scene")
if not cli.wait_for_service(timeout_sec=10.0):
    print("/get_planning_scene not available (is move_group running?)")
    raise SystemExit(1)
req = GetPlanningScene.Request()
req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
fut = cli.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
if not fut.done():
    print("no answer from /get_planning_scene")
    raise SystemExit(1)
acm = fut.result().scene.allowed_collision_matrix
names = list(acm.entry_names)
print(f"matrix has {len(names)} entries; gripper links present: {[g for g in ('gripper_palm', 'left_finger', 'right_finger', 'gripper_tcp') if g in names]}")


def allowed(a, b):
    if a in names and b in names:
        return bool(acm.entry_values[names.index(a)].enabled[names.index(b)])
    return None


for a, b in (("gripper_palm", "wrist_3_link"), ("gripper_palm", "tool0"), ("gripper_palm", "left_finger"), ("left_finger", "right_finger")):
    print(f"   collision allowed {a} <-> {b}: {allowed(a, b)}")
