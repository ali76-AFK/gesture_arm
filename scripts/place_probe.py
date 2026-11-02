#!/usr/bin/env python3
"""place_probe (READ-ONLY, no motion): dry run of the checks for "pick OBJECT, put it down at GOAL, released DROP_H above the surface".
For each goal (world x,y in base_link) it prints, from the live depth image and /detections:
 - the support height under the object's footprint at the goal (highest surface, measured from depth), what it is, and how much of the footprint is supported
 - the fingertip-frame height at release = support + DROP_H + hang  (hang = grasp height - object bottom, grasp as in pick_place.py)
 - the clearance ring around the object and around the open fingers, checked in the depth image
 - collision-checked IK at the pick and at the goal (transit and release heights)
usage: place_probe.py OBJECT X,Y [X,Y ...]     Runs INSIDE the sim container (run.sh exec). Arm at home; sim, arm_control, object_detector running.
Gripper numbers come from urdf/gripper.xacro (open fingers: inner faces +-0.0375 m, outer faces +-0.0525 m along the closing axis, finger width 0.02 m,
lowest finger point 0.007 m below the fingertip frame). The closing axis is world x ONLY IF the tool frame matches QUAT (verify with tf2_echo)."""
import json
import math
import sys
import time
import numpy as np

CAM = (0.6, 0.0, 1.25)                     # camera position in base_link, looking straight down
TABLE = (0.20, 1.00, -0.50, 0.50)          # x_min, x_max, y_min, y_max of the table top (world file)
BIN_C, BIN_INNER = (0.55, -0.33), 0.07     # bin centre and inner half width (world file: outer 0.08, wall 0.01)
TABLE_TOP, BIN_FLOOR = 0.20, 0.21
OBJ_H = {"red_cube": 0.05, "green_cube": 0.05, "blue_cylinder": 0.06, "yellow_block": 0.08}   # heights from the world file
DROP_H = 0.05                              # gap between the object's bottom and the surface at release (your request)
GRASP_MIN, GRASP_BELOW_TOP, APPROACH, TRANSIT_Z = TABLE_TOP + 0.026, 0.04, 0.12, 0.40        # as in pick_place.py
FING_IN, FING_OUT, FING_HALF, FING_BELOW = 0.0375, 0.0525, 0.010, 0.007                      # gripper.xacro, open
MARGIN, VMARGIN = 0.010, 0.005             # horizontal / vertical clearance margins (my choice, printed, to be tuned)
CELL, MIN_CELLS, FRAC_TOL, FRAC_WARN, FRAC_FAIL, UNSEEN_WARN = 0.003, 10, 0.004, 0.85, 0.50, 0.25   # height-grid cell (m), thresholds
TCP_OFFSET = 0.115
QUAT = (0.7071068, -0.7071068, 0.0, 0.0)   # x, y, z, w: tool pointing down (as pick_place.py)
REST = [-1.4, 1.3, -1.47, -1.5708, 0.0]
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


GX0, GY0, GX1, GY1 = 0.18, -0.52, 1.02, 0.52     # height grid covers the table plus a little


def world_points(depth, K, cam=CAM):
    """Every depth pixel -> base_link (X, Y, Z) plus a validity mask (same camera model as object_detector.py / goto_point.py)."""
    h, w = depth.shape
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    v, u = np.mgrid[0:h, 0:w]
    d = depth.astype(np.float64)
    ok = np.isfinite(d) & (d > 0.05)
    d = np.where(ok, d, 0.0)
    return cam[0] - (v - cy) * d / fy, cam[1] - (u - cx) * d / fx, cam[2] - d, ok


def height_grid(pts):
    """Top-down height field: for every CELL x CELL cell the highest measured surface point (-inf = never seen).
    Using the maximum removes the side faces and shadows a tilted view sees, so only upward-facing surfaces count."""
    X, Y, Z, ok = pts
    nx, ny = int(round((GX1 - GX0) / CELL)), int(round((GY1 - GY0) / CELL))
    ix, iy = np.floor((X - GX0) / CELL).astype(int), np.floor((Y - GY0) / CELL).astype(int)
    m = ok & (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    g = np.full((nx, ny), -np.inf)
    np.maximum.at(g, (ix[m], iy[m]), Z[m])
    return g


def box(cx, cy, hx, hy):
    return (cx - hx, cx + hx, cy - hy, cy + hy)


def _idx(b):
    """Index ranges of the cells whose CENTRE lies inside box b (so a box is not widened by up to one cell on each side)."""
    return (max(int(math.ceil((b[0] - GX0) / CELL - 0.5)), 0), int(math.floor((b[1] - GX0) / CELL - 0.5)) + 1,
            max(int(math.ceil((b[2] - GY0) / CELL - 0.5)), 0), int(math.floor((b[3] - GY0) / CELL - 0.5)) + 1)


def region(grid, inc, exc=()):
    """Heights of the cells of box inc, minus the cells of the boxes in exc. Returns (seen heights, total cell count)."""
    i0, i1, j0, j1 = _idx(inc)
    sub = grid[i0:i1, j0:j1]
    keep = np.ones(sub.shape, bool)
    for e in exc:
        e0, e1, f0, f1 = _idx(e)
        keep[max(e0 - i0, 0):max(e1 - i0, 0), max(f0 - j0, 0):max(f1 - j0, 0)] = False
    vals = sub[keep]
    return vals[np.isfinite(vals)], int(vals.size)


def overlap(a, b):
    return a[0] < b[1] and a[1] > b[0] and a[2] < b[3] and a[3] > b[2]


def pick_numbers(name, o):
    z_grasp = max(GRASP_MIN, o["top_z"] - GRASP_BELOW_TOP)
    bottom = o["top_z"] - OBJ_H[name]
    return z_grasp, bottom, z_grasp - bottom


def classify(gx, gy, sup, objs, skip):
    for n, o in objs.items():
        if n != skip and abs(gx - o["x"]) <= o["wx"] / 2 + 0.005 and abs(gy - o["y"]) <= o["wy"] / 2 + 0.005 and abs(sup - o["top_z"]) < 0.008:
            return "on top of " + n
    if abs(gx - BIN_C[0]) <= BIN_INNER and abs(gy - BIN_C[1]) <= BIN_INNER and abs(sup - BIN_FLOOR) < 0.006:
        return "bin floor"
    if abs(sup - TABLE_TOP) < 0.006:
        return "table"
    return "unclassified surface"


def evaluate_goal(goal, name, objs, grid, ik=None, margin=MARGIN):
    """Returns (lines, fails, warns). ik(x, y, z_fingertip) -> True/False/None, or None to skip IK."""
    gx, gy = goal
    o = objs[name]
    hx, hy = o["wx"] / 2.0, o["wy"] / 2.0
    z_grasp, bottom, hang = pick_numbers(name, o)
    fails, warns = [], []
    out = ["goal x=%.3f y=%.3f for %s: footprint %.3f x %.3f m | grasp z %.3f, object bottom %.3f, hang below fingertip frame %.3f"
           % (gx, gy, name, o["wx"], o["wy"], z_grasp, bottom, hang)]

    def verdict():
        out.append("   verdict: " + ("FAIL: " + "; ".join(fails) if fails else "OK") + (" | WARN: " + "; ".join(warns) if warns else ""))
        return out, fails, warns

    foot = box(gx, gy, hx, hy)
    own = box(o["x"], o["y"], hx + 0.005, hy + 0.005)
    if not (TABLE[0] <= foot[0] and foot[1] <= TABLE[1] and TABLE[2] <= foot[2] and foot[3] <= TABLE[3]):
        fails.append("footprint leaves the table")
    if overlap(foot, own):
        fails.append("goal overlaps the spot where the object stands now (excluded from the first version)")
    if fails:
        return verdict()
    z, tot = region(grid, foot)
    if z.size < MIN_CELLS:
        fails.append("only %d of %d footprint cells seen by the camera" % (z.size, tot))
        return verdict()
    sup = float(np.percentile(z, 99))
    frac = float(np.sum(np.abs(z - sup) <= FRAC_TOL)) / float(tot)     # hidden cells count as NOT supported
    unseen = 1.0 - z.size / float(tot)
    what = classify(gx, gy, sup, objs, name)
    z_rel = sup + DROP_H + hang
    out.append("   support z=%.3f (%s) | %d of %d cells seen | supported fraction %.2f (hidden = unsupported) | median z %.3f"
               % (sup, what, z.size, tot, frac, float(np.median(z))))
    out.append("   release: fingertip frame z=%.3f = support %.3f + drop %.3f + hang %.3f" % (z_rel, sup, DROP_H, hang))
    if unseen > UNSEEN_WARN:
        warns.append("%.0f%% of the footprint is hidden from the camera" % (100 * unseen))
    if what == "unclassified surface":
        fails.append("unclassified surface at z=%.3f" % sup)
    if frac < FRAC_FAIL:
        fails.append("only %.0f%% of the footprint is supported" % (100 * frac))
    elif frac < FRAC_WARN:
        warns.append("only %.0f%% of the footprint is supported (overhang)" % (100 * frac))
    lim = sup + DROP_H - VMARGIN
    ring, rtot = region(grid, box(gx, gy, hx + margin, hy + margin), [foot, own])
    if ring.size < MIN_CELLS:
        warns.append("object ring not measurable (%d of %d cells seen)" % (ring.size, rtot))
    else:
        rm = float(np.percentile(ring, 99))
        out.append("   object ring (%.0f mm): max z %.3f, limit %.3f, %d of %d cells seen -> %s"
                   % (margin * 1000, rm, lim, ring.size, rtot, "ok" if rm <= lim else "CLASH"))
        if rm > lim:
            fails.append("something up to z=%.3f within %.0f mm of the object's footprint" % (rm, margin * 1000))
        if 1.0 - ring.size / float(rtot) > UNSEEN_WARN:
            warns.append("%.0f%% of the object ring is hidden from the camera" % (100 * (1.0 - ring.size / float(rtot))))
    flim = z_rel - FING_BELOW - VMARGIN
    for s, nm in ((1, "+x finger"), (-1, "-x finger")):
        xa, xb = gx + s * (FING_IN - margin), gx + s * (FING_OUT + margin)
        fz, ftot = region(grid, (min(xa, xb), max(xa, xb), gy - FING_HALF - margin, gy + FING_HALF + margin), [own])
        if fz.size < MIN_CELLS:
            warns.append("%s box not measurable (%d of %d cells seen)" % (nm, fz.size, ftot))
            continue
        fm = float(np.percentile(fz, 99))
        out.append("   %s box: max z %.3f, limit %.3f, %d of %d cells seen -> %s" % (nm, fm, flim, fz.size, ftot, "ok" if fm <= flim else "CLASH"))
        if fm > flim:
            fails.append("%s would meet something at z=%.3f" % (nm, fm))
    tall = max(v["top_z"] for k, v in objs.items() if k != name)
    low = TRANSIT_Z - hang
    out.append("   carry: lowest point of the held object at transit z=%.3f, tallest other top %.3f -> margin %.3f m" % (low, tall, low - tall))
    if low - tall < 0.03:
        fails.append("transit height too low for the held object")
    if z_rel > TRANSIT_Z - 0.02:
        warns.append("release height %.3f is within 2 cm of the transit height %.2f" % (z_rel, TRANSIT_Z))
    if ik is not None:
        tests = [("pick pre-grasp", o["x"], o["y"], z_grasp + APPROACH), ("pick", o["x"], o["y"], z_grasp),
                 ("goal transit", gx, gy, TRANSIT_Z), ("goal release", gx, gy, z_rel)]
        res = [(t[0], t[3], ik(t[1], t[2], t[3])) for t in tests]
        out.append("   IK (collision-checked, tool down): " + " | ".join("%s z=%.3f %s" % r for r in res))
        for r in res:
            if r[2] is not True:
                fails.append("IK %s at %s z=%.3f" % ("failed" if r[2] is False else "unavailable", r[0], r[1]))
    return verdict()


def make_ik(node, cli):
    from builtin_interfaces.msg import Duration
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    import rclpy

    def ik(x, y, z_tcp):
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
    return ik


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from moveit_msgs.srv import GetPositionIK
    if len(sys.argv) < 3 or sys.argv[1] not in OBJ_H:
        print("usage: place_probe.py OBJECT X,Y [X,Y ...]   OBJECT in %s" % sorted(OBJ_H))
        return
    name = sys.argv[1]
    goals = [tuple(float(v) for v in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    n = Node("place_probe")
    box_ = {}
    n.create_subscription(Image, "/camera/depth_image",
                          lambda m: box_.update(depth=np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()), 1)
    n.create_subscription(CameraInfo, "/camera/camera_info", lambda m: box_.update(K=list(m.k)), 1)
    n.create_subscription(String, "/detections", lambda m: box_.update(det=json.loads(m.data)), 1)
    cli = n.create_client(GetPositionIK, "/compute_ik")
    t0 = time.time()
    while not all(k in box_ for k in ("depth", "K", "det")) and time.time() - t0 < 10.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    missing = [k for k in ("depth", "K", "det") if k not in box_]
    if missing:
        print("missing inputs: %s (sim, bridge, object_detector running?)" % missing)
        return
    objs = box_["det"].get("objs", {})
    if name not in objs:
        print("%s is not in /detections %s (arm at home? objects reset?)" % (name, sorted(objs)))
        return
    ik = make_ik(n, cli) if cli.wait_for_service(timeout_sec=5.0) else None
    print("depth %dx%d, K fx=%.1f fy=%.1f cx=%.1f cy=%.1f | detections %s | IK %s | margins: horizontal %.0f mm, vertical %.0f mm"
          % (box_["depth"].shape[1], box_["depth"].shape[0], box_["K"][0], box_["K"][4], box_["K"][2], box_["K"][5],
             sorted(objs), "ready" if ik else "NOT available", MARGIN * 1000, VMARGIN * 1000))
    grid = height_grid(world_points(box_["depth"], box_["K"]))
    for g in goals:
        print("\n".join(evaluate_goal(g, name, objs, grid, ik)[0]))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
