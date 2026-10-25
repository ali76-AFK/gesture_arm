#!/usr/bin/env python3
"""reach_map v2: multi-seed IK reachability of the UR5e with the tool pointing down, at several fingertip heights (read-only).
usage: reach_map.py [fingertip_height_m ...]   default: 0.23 0.30 0.38"""
import collections
import math
import sys
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity

QUAT = (0.7071068, -0.7071068, 0.0, 0.0)          # x, y, z, w: tool pointing down, same as pick_place.py
TCP_OFFSET = 0.115
HEIGHTS = [float(a) for a in sys.argv[1:]] or [0.23, 0.30, 0.38]
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
REST = [-1.4, 1.3, -1.47, -1.5708, 0.0]           # lift, elbow, wrist 1, 2, 3 of the 'ready' pose

rclpy.init()
n = Node("reach_map")
ik_cli = n.create_client(GetPositionIK, "/compute_ik")
val_cli = n.create_client(GetStateValidity, "/check_state_validity")
if not (ik_cli.wait_for_service(timeout_sec=15.0) and val_cli.wait_for_service(timeout_sec=15.0)):
    print("MoveIt services not available: start the simulation and arm_control first")
    raise SystemExit(1)


def call(cli, req, t=3.0):
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=t)
    return fut.result() if fut.done() else None


def ik(x, y, z_tcp, avoid, pan):
    req = GetPositionIK.Request()
    r = req.ik_request
    r.group_name, r.ik_link_name, r.avoid_collisions = "ur_manipulator", "tool0", avoid
    r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=[pan] + REST))
    r.timeout = Duration(sec=0, nanosec=150_000_000)
    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z_tcp + TCP_OFFSET
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = QUAT
    r.pose_stamped = ps
    res = call(ik_cli, req)
    return res.solution if (res is not None and res.error_code.val == 1) else None


def cell(x, y, z):
    base = math.atan2(y, x) - math.asin(min(0.138 / max(math.hypot(x, y), 0.14), 1.0))
    seeds = [base, base - 0.35, base + 0.35]
    for pan in seeds:
        if ik(x, y, z, True, pan) is not None:
            return "#", None
    for pan in seeds:
        sol = ik(x, y, z, False, pan)
        if sol is not None:
            req = GetStateValidity.Request()
            req.robot_state, req.group_name = sol, "ur_manipulator"
            v = call(val_cli, req)
            if v is not None and v.valid:
                return "#", None
            pairs = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in (v.contacts if v else [])})
            return "o", pairs
    return ".", None


xs = [round(1.00 - 0.05 * i, 2) for i in range(17)]
ys = [round(0.6 - 0.1 * j, 2) + 0.0 for j in range(13)]
for z in HEIGHTS:
    print(f"\n===== fingertip height z={z:.2f} m, tool pointing down, 3 IK seeds per cell =====")
    print("rows: x from far (top) to near the base | columns: y from +0.6 (left) to -0.6 (right), like the overhead camera image")
    print("# reachable (collision-checked) | o found only when collisions are ignored | . no solution found\n")
    print("        " + "".join(f"{y:+.1f}"[0] + f"{y:+.1f}"[2:] for y in ys) + "   (y in m)")
    count, pairs_seen = {"#": 0, "o": 0, ".": 0}, collections.Counter()
    for x in xs:
        row = []
        for y in ys:
            c, pairs = cell(x, y, z)
            count[c] += 1
            row.append(c)
            for p in pairs or []:
                pairs_seen[p] += 1
        print(f"x={x:4.2f}  " + "  ".join(row), flush=True)
    print("cells:", count)
    if pairs_seen:
        print("collisions behind the 'o' cells (pair: number of cells):")
        for p, k in pairs_seen.most_common(8):
            print(f"   {p[0]} <-> {p[1]}: {k}")
print("\nsanity check at z=0.23, positions where picks and drops already worked:")
for name, (x, y) in {"red_cube": (0.45, 0.15), "green_cube": (0.65, 0.18), "blue_cylinder": (0.55, -0.08),
                     "yellow_block": (0.70, -0.12), "bin centre": (0.55, -0.33)}.items():
    print(f"   {name:14s} ({x:.2f},{y:+.2f}): {'reachable' if cell(x, y, 0.23)[0] == '#' else 'NOT FOUND by this probe'}")
