#!/usr/bin/env python3
"""probe_goto_ik (READ-ONLY, no motion): asks MoveIt /compute_ik, collision checking on, for fingertip targets,
once with no seed (current state) and once per pan seed. Usage: probe_goto_ik.py [x,y,z ...]"""
import sys
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK

J = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
REST = [-1.4, 1.3, -1.47, -1.5708, 0.0]
QUAT = (0.7071068, -0.7071068, 0.0, 0.0)
TCP = 0.115
DEFAULT = [(0.55, 0.0, 0.35), (0.648, 0.19, 0.35), (0.45, 0.30, 0.35), (0.70, -0.30, 0.35)]
PANS = [-1.5, -0.75, 0.0, 0.75, 1.5]


def ik(node, cli, p, pan):
    req = GetPositionIK.Request()
    r = req.ik_request
    r.group_name = "ur_manipulator"
    r.ik_link_name = "tool0"
    r.avoid_collisions = True
    r.timeout = Duration(sec=0, nanosec=150_000_000)
    r.pose_stamped.header.frame_id = "base_link"
    r.pose_stamped.pose.position.x = float(p[0])
    r.pose_stamped.pose.position.y = float(p[1])
    r.pose_stamped.pose.position.z = float(p[2]) + TCP
    r.pose_stamped.pose.orientation.x = QUAT[0]
    r.pose_stamped.pose.orientation.y = QUAT[1]
    r.pose_stamped.pose.orientation.z = 0.0
    r.pose_stamped.pose.orientation.w = 0.0   # QUAT = (x 0.7071068, y -0.7071068, z 0, w 0), as pick_place.py
    if pan is not None:
        r.robot_state.joint_state.name = list(J)
        r.robot_state.joint_state.position = [float(pan)] + REST
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    if not fut.done():
        return None, "no reply"
    res = fut.result()
    if res.error_code.val != 1:
        return None, "error_code=%d" % res.error_code.val
    pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [round(pos[j], 3) for j in J], "ok"


def main():
    targets = [tuple(float(v) for v in a.split(",")) for a in sys.argv[1:]] or DEFAULT
    rclpy.init()
    node = Node("probe_goto_ik")
    cli = node.create_client(GetPositionIK, "/compute_ik")
    if not cli.wait_for_service(timeout_sec=5.0):
        print("no /compute_ik service")
        return
    for p in targets:
        print("target fingertip x=%.3f y=%.3f z=%.3f (tool0 z=%.3f)" % (p[0], p[1], p[2], p[2] + TCP))
        for pan in [None] + PANS:
            sol, msg = ik(node, cli, p, pan)
            print("   seed pan=%-5s %s %s" % ("none" if pan is None else pan, msg, sol if sol else ""))
    node.destroy_node()
    rclpy.shutdown()


main()
