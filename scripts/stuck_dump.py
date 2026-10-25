#!/usr/bin/env python3
"""stuck_dump: read-only snapshot of why the arm may not respond. Run it WHILE the arm is stuck. It sends no motion command."""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from control_msgs.msg import JointTrajectoryControllerState

rclpy.init()
n = Node("stuck_dump")
got = {}
n.create_subscription(JointState, "/joint_states", lambda m: got.update(js=m), 10)
n.create_subscription(String, "/arm_state", lambda m: got.update(arm=m.data), 10)
n.create_subscription(String, "/pick_place_state", lambda m: got.update(pick=m.data), 10)
n.create_subscription(JointTrajectoryControllerState, "/scaled_joint_trajectory_controller/controller_state",
                      lambda m: got.update(cs=m), 10)
t = time.time()
while time.time() - t < 4.0:
    rclpy.spin_once(n, timeout_sec=0.1)
print("arm_state            :", got.get("arm", "(no message)"))
print("pick_place_state     :", got.get("pick", "(no message)"))
cs = got.get("cs")
if cs is not None:
    print("controller error (rad):", [round(v, 3) for v in cs.error.positions])
    print("controller reference  :", [round(v, 3) for v in cs.reference.positions])
else:
    print("controller_state     : (no message)")
js = got.get("js")
if js is None:
    print("no /joint_states"); raise SystemExit(1)
print("joints               :", {k: round(v, 3) for k, v in zip(js.name, js.position)})
cli = n.create_client(GetStateValidity, "/check_state_validity")
if not cli.wait_for_service(timeout_sec=5.0):
    print("/check_state_validity not available"); raise SystemExit(1)
req = GetStateValidity.Request()
req.robot_state = RobotState(joint_state=js)
req.group_name = "ur_manipulator"
fut = cli.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
res = fut.result()
if res is None:
    print("check_state_validity: no answer")
else:
    print("current state valid  :", res.valid)
    for c in res.contacts[:12]:
        print(f"   contact: {c.contact_body_1} <-> {c.contact_body_2}  depth {c.depth:.4f} m")
    if not res.contacts:
        print("   (no contacts reported)")
