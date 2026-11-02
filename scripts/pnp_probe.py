#!/usr/bin/env python3
"""pnp_probe (READ-ONLY, no motion): for candidate place goals (world x,y) prints, with the arm at home and the objects visible:
 - inside the table rectangle / inside the bin footprint (constants below, from the project handover; compare with the SDF)
 - distance from the goal to every object in the latest /detections (centre distance and point-to-top-face-box gap)
 - collision-checked IK reachability (tool pointing down, 3 seeds, same method as goto_point.py) at several fingertip heights
usage: pnp_probe.py X,Y [X,Y ...]       Runs INSIDE the sim container (run.sh exec). Needs sim + arm_control + object_detector."""
import json
import math
import sys
import time

TABLE = (0.20, 1.00, -0.50, 0.50)          # x_min, x_max, y_min, y_max of the table top (handover, pixel_world_check)
BIN = (0.47, 0.63, -0.41, -0.25)           # bin outer box x_min, x_max, y_min, y_max (handover, world description)
HEIGHTS = (0.23, 0.26, 0.30, 0.35)         # fingertip z (m) to test
TCP_OFFSET = 0.115
QUAT = (0.7071068, -0.7071068, 0.0, 0.0)   # x, y, z, w: tool pointing down (as pick_place.py and goto_point.py)
REST = [-1.4, 1.3, -1.47, -1.5708, 0.0]
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def inside(x, y, box):
    return box[0] <= x <= box[1] and box[2] <= y <= box[3]


def box_gap(gx, gy, o):
    """Distance from the point (gx, gy) to the object's axis-aligned top-face box (wx along world x, wy along world y)."""
    hx, hy = o["wx"] / 2.0, o["wy"] / 2.0
    return math.hypot(max(abs(gx - o["x"]) - hx, 0.0), max(abs(gy - o["y"]) - hy, 0.0))


def report(goal, objs):
    gx, gy = goal
    lines = ["goal x=%.3f y=%.3f: in table %s | in bin footprint %s" % (gx, gy, inside(gx, gy, TABLE), inside(gx, gy, BIN))]
    if not objs:
        lines.append("   no objects in /detections (arm in view? sim running? object_detector running?)")
    for name, o in sorted(objs.items()):
        lines.append("   %-14s at (%.3f, %.3f) top z %.3f size %.3f x %.3f m | centre distance %.3f m | gap to its top face %.3f m"
                     % (name, o["x"], o["y"], o["top_z"], o["wx"], o["wy"], math.hypot(gx - o["x"], gy - o["y"]), box_gap(gx, gy, o)))
    return "\n".join(lines)


def reach(node, cli, x, y, z_tcp):
    from builtin_interfaces.msg import Duration
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    import rclpy
    base = math.atan2(y, x) - math.asin(min(0.138 / max(math.hypot(x, y), 0.14), 1.0))
    for pan in (base, base - 0.35, base + 0.35):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name, r.ik_link_name, r.avoid_collisions = "ur_manipulator", "tool0", True
        r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=[pan] + REST))
        r.timeout = Duration(sec=0, nanosec=150_000_000)
        r.pose_stamped.header.frame_id = "base_link"
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y = x, y
        r.pose_stamped.pose.position.z = z_tcp + TCP_OFFSET
        (r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y,
         r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w) = QUAT
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
        if fut.done() and fut.result().error_code.val == 1:
            return True
    return False


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from moveit_msgs.srv import GetPositionIK
    goals = [tuple(float(v) for v in a.split(",")) for a in sys.argv[1:]]
    if not goals:
        print("usage: pnp_probe.py X,Y [X,Y ...]")
        return
    rclpy.init()
    n = Node("pnp_probe")
    box = {}
    n.create_subscription(String, "/detections", lambda m: box.update(d=json.loads(m.data)), 1)
    cli = n.create_client(GetPositionIK, "/compute_ik")
    t0 = time.time()
    while "d" not in box and time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    objs = (box.get("d") or {}).get("objs", {})
    ik_ok = cli.wait_for_service(timeout_sec=5.0)
    print("detections: %s | /compute_ik: %s" % (sorted(objs) or "none", "ready" if ik_ok else "NOT available"))
    for g in goals:
        print(report(g, objs))
        if ik_ok:
            print("   reachable (collision-checked IK, tool down): " + " | ".join(
                "fingertip z=%.2f %s" % (z, reach(n, cli, g[0], g[1], z)) for z in HEIGHTS))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
