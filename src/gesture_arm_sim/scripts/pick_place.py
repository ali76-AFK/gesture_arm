#!/usr/bin/env python3
"""pick_place: hard-coded objects -> top-down pick, carry, drop into the bin."""
import json, math, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from sensor_msgs.msg import CameraInfo, Image, JointState
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint, PlanningOptions,
                             PositionConstraint, OrientationConstraint, BoundingVolume, RobotState)

GROUP, FRAME, TOOL = "ur_manipulator", "base_link", "tool0"
TCP_OFFSET = 0.115                      # tool0 -> gripper_tcp along the tool z axis (pointing down)
QUAT = (0.7071068, -0.7071068, 0.0, 0.0)  # tool pointing down (measured in the ready pose)
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
READY = [0.0, -1.4, 1.3, -1.47, -1.5708, 0.0]
FINGERS = ["left_finger_joint", "right_finger_joint"]

# name: (x, y, grasp height of object centre, finger travel when closing)  -- from make_world.py
OBJECTS = {
    "red_cube":      (0.45,  0.15, 0.226, 0.018),
    "green_cube":    (0.65,  0.18, 0.226, 0.018),
    "blue_cylinder": (0.55, -0.08, 0.231, 0.018),
    "yellow_block":  (0.70, -0.12, 0.241, 0.022),
}
BIN = (0.55, -0.33)
BIN_HALF = 0.08                    # half width of the bin footprint (m)
TABLE_TOP = 0.20
HOME = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]    # look pose: arm straight up, camera sees the whole table
APPROACH, LIFT, TRANSIT_Z, DROP_Z = 0.12, 0.15, 0.40, 0.27   # heights of the fingertip frame (m)


# ---------- place goal: geometry. Pure functions, no ROS (same logic as scripts/place_probe.py, checked on the real depth image on 31 Oct) ----------
CAM = (0.6, 0.0, 1.25)                     # overhead camera position in base_link, looking straight down
TABLE = (0.20, 1.00, -0.50, 0.50)          # x_min, x_max, y_min, y_max of the table top (world file)
BIN_INNER, BIN_FLOOR = 0.07, 0.21          # bin inner half width and floor top (world file)
OBJ_H = {"red_cube": 0.05, "green_cube": 0.05, "blue_cylinder": 0.06, "yellow_block": 0.08}   # object heights (world file)
PLACE_GAP = 0.05                           # gap between the object's bottom and the surface at release (m)
FING_IN, FING_OUT, FING_HALF, FING_BELOW = 0.0375, 0.0525, 0.010, 0.007   # open fingers (gripper.xacro); closing axis = world x (tf2_echo, 31 Oct)
MARGIN, VMARGIN = 0.010, 0.005             # horizontal / vertical clearance margins (m), tunable
CELL, MIN_CELLS, FRAC_TOL, FRAC_WARN, FRAC_FAIL, UNSEEN_WARN = 0.003, 10, 0.004, 0.85, 0.50, 0.25
GX0, GY0, GX1, GY1 = 0.18, -0.52, 1.02, 0.52   # height grid covers the table plus a little
SNAP_MARGIN, SNAP_DZ = 0.010, 0.015        # a click this close to another object's top face snaps to its centre
REST = READY[1:]                           # IK seeds: lift, elbow, wrist 1, 2, 3 of the ready pose


def parse_cmd(text):
    """'red_cube' -> ('red_cube', None); '{"object": "red_cube", "goal": {"x": 0.45, "y": 0.3}}' -> ('red_cube', (0.45, 0.3))."""
    t = text.strip()
    if not t.startswith("{"):
        return t.lower(), None
    d = json.loads(t)
    g = d.get("goal")
    return str(d["object"]).strip().lower(), ((float(g["x"]), float(g["y"])) if g else None)


def grasp_z(top):
    return max(TABLE_TOP + 0.026, top - 0.04)      # grasp height of the fingertip frame, as in sequence()


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
    The maximum removes the side faces and shadows a tilted view sees, so only upward-facing surfaces count."""
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
    """Index ranges of the cells whose CENTRE lies inside box b."""
    return (max(int(math.ceil((b[0] - GX0) / CELL - 0.5)), 0), int(math.floor((b[1] - GX0) / CELL - 0.5)) + 1,
            max(int(math.ceil((b[2] - GY0) / CELL - 0.5)), 0), int(math.floor((b[3] - GY0) / CELL - 0.5)) + 1)


def region(grid, inc, exc=()):
    """Heights of the cells of box inc minus the cells of the boxes in exc. Returns (seen heights, total cell count)."""
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


def classify(gx, gy, sup, objs, skip):
    for n, o in objs.items():
        if n != skip and abs(gx - o["x"]) <= o["wx"] / 2 + 0.005 and abs(gy - o["y"]) <= o["wy"] / 2 + 0.005 and abs(sup - o["top_z"]) < 0.008:
            return "on top of " + n
    if abs(gx - BIN[0]) <= BIN_INNER and abs(gy - BIN[1]) <= BIN_INNER and abs(sup - BIN_FLOOR) < 0.006:
        return "bin floor"
    if abs(sup - TABLE_TOP) < 0.006:
        return "table"
    return "unclassified surface"


def click_to_world(u, v, depth, K, cam=CAM, half=2):
    """Clicked pixel -> base_link point of the surface there (median depth of a small window)."""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    h, w = depth.shape
    iu, iv = int(round(u)), int(round(v))
    if not (0 <= iu < w and 0 <= iv < h):
        return None
    win = depth[max(0, iv - half):iv + half + 1, max(0, iu - half):iu + half + 1]
    win = win[np.isfinite(win) & (win > 0.05)]
    if win.size < 5:
        return None
    d = float(np.median(win))
    return (cam[0] - (v - cy) * d / fy, cam[1] - (u - cx) * d / fx, cam[2] - d)


def snap_goal(pt, objs, skip):
    """If the clicked surface point lies on another object's top face, return (x, y, [u, v], name) of that object's centre, else None."""
    for n, o in objs.items():
        if n != skip and abs(pt[0] - o["x"]) <= o["wx"] / 2 + SNAP_MARGIN and abs(pt[1] - o["y"]) <= o["wy"] / 2 + SNAP_MARGIN \
                and abs(pt[2] - o["top_z"]) < SNAP_DZ:
            return o["x"], o["y"], list(o["px"]), n
    return None


def evaluate_goal(goal, name, objs, grid, ik=None, margin=MARGIN):
    """-> (lines, fails, warns, info). ik(x, y, z_fingertip) -> True / False / None (unknown), or None to skip the IK checks."""
    gx, gy = goal
    o = objs[name]
    hx, hy = o["wx"] / 2.0, o["wy"] / 2.0
    z_grasp = grasp_z(o["top_z"])
    bottom = o["top_z"] - OBJ_H[name]
    hang = z_grasp - bottom                     # how far the object's bottom hangs below the fingertip frame
    fails, warns, info = [], [], {}
    out = ["goal x=%.3f y=%.3f for %s: footprint %.3f x %.3f m | grasp z %.3f, object bottom %.3f, hang below fingertip frame %.3f"
           % (gx, gy, name, o["wx"], o["wy"], z_grasp, bottom, hang)]

    def done():
        out.append("   verdict: " + ("FAIL: " + "; ".join(fails) if fails else "OK") + (" | WARN: " + "; ".join(warns) if warns else ""))
        return out, fails, warns, info

    foot = box(gx, gy, hx, hy)
    own = box(o["x"], o["y"], hx + 0.005, hy + 0.005)      # where the object stands now; it will have left
    if not (TABLE[0] <= foot[0] and foot[1] <= TABLE[1] and TABLE[2] <= foot[2] and foot[3] <= TABLE[3]):
        fails.append("footprint leaves the table")
    if overlap(foot, own):
        fails.append("goal overlaps the spot where the object stands now")
    if fails:
        return done()
    z, tot = region(grid, foot)
    if z.size < MIN_CELLS:
        fails.append("only %d of %d footprint cells seen by the camera" % (z.size, tot))
        return done()
    sup = float(np.percentile(z, 99))
    frac = float(np.sum(np.abs(z - sup) <= FRAC_TOL)) / float(tot)     # hidden cells count as NOT supported
    what = classify(gx, gy, sup, objs, name)
    z_rel = sup + PLACE_GAP + hang
    info.update(support=sup, what=what, frac=frac, z_release=z_rel, hang=hang)
    out.append("   support z=%.3f (%s) | %d of %d cells seen | supported fraction %.2f (hidden = unsupported)" % (sup, what, z.size, tot, frac))
    out.append("   release: fingertip frame z=%.3f = support %.3f + gap %.3f + hang %.3f" % (z_rel, sup, PLACE_GAP, hang))
    if 1.0 - z.size / float(tot) > UNSEEN_WARN:
        warns.append("%.0f%% of the footprint is hidden from the camera" % (100 * (1.0 - z.size / float(tot))))
    if what == "unclassified surface":
        fails.append("unclassified surface at z=%.3f (arm in the way?)" % sup)
    if frac < FRAC_FAIL:
        fails.append("only %.0f%% of the footprint is supported" % (100 * frac))
    elif frac < FRAC_WARN:
        warns.append("only %.0f%% of the footprint is supported (overhang)" % (100 * frac))
    lim = sup + PLACE_GAP - VMARGIN
    ring, rtot = region(grid, box(gx, gy, hx + margin, hy + margin), [foot, own])
    if ring.size < MIN_CELLS:
        warns.append("object ring not measurable (%d of %d cells seen)" % (ring.size, rtot))
    else:
        rm = float(np.percentile(ring, 99))
        out.append("   object ring (%.0f mm): max z %.3f, limit %.3f, %d of %d cells seen -> %s" % (margin * 1000, rm, lim, ring.size, rtot, "ok" if rm <= lim else "CLASH"))
        if rm > lim:
            fails.append("something up to z=%.3f within %.0f mm of the object's footprint" % (rm, margin * 1000))
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
    tall = max([v["top_z"] for k, v in objs.items() if k != name], default=TABLE_TOP)
    low = TRANSIT_Z - hang
    out.append("   carry: lowest point of the held object at transit z=%.3f, tallest other top %.3f -> margin %.3f m" % (low, tall, low - tall))
    if low - tall < 0.03:
        fails.append("transit height too low for the held object")
    if z_rel > TRANSIT_Z - 0.01:
        fails.append("release height %.3f is too close to the transit height %.2f" % (z_rel, TRANSIT_Z))
    if ik is not None:
        tests = [("pick pre-grasp", o["x"], o["y"], z_grasp + APPROACH), ("pick", o["x"], o["y"], z_grasp),
                 ("goal transit", gx, gy, TRANSIT_Z), ("goal release", gx, gy, z_rel)]
        res = [(t[0], t[3], ik(t[1], t[2], t[3])) for t in tests]
        out.append("   IK (collision-checked, tool down): " + " | ".join("%s z=%.3f %s" % r for r in res))
        for r in res:
            if r[2] is not True:
                fails.append("IK %s at %s z=%.3f" % ("failed" if r[2] is False else "unavailable", r[0], r[1]))
    return done()


class Abort(Exception):
    pass


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        cb = ReentrantCallbackGroup()
        self.mg = ActionClient(self, MoveGroup, "/move_action", callback_group=cb)
        self.ex = ActionClient(self, ExecuteTrajectory, "/execute_trajectory", callback_group=cb)
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 "/gripper_controller/follow_joint_trajectory", callback_group=cb)
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path", callback_group=cb)
        self.state_pub = self.create_publisher(String, "/pick_place_state", 10)
        self.create_subscription(String, "/pick_place_command", self.on_cmd, 10, callback_group=cb)
        self.create_subscription(String, "/arm_command", self.on_arm_cmd, 10, callback_group=cb)
        self.create_subscription(String, "/detections", self.on_det, 10, callback_group=cb)
        self.det = {}
        self.abort, self.busy, self.cur = threading.Event(), False, None
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik", callback_group=cb)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, 1, callback_group=cb)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 1, callback_group=cb)
        self.create_subscription(String, "/place/click", self.on_place_click, 10, callback_group=cb)
        self.create_subscription(String, "/place/clear", self.on_place_clear, 10, callback_group=cb)
        self.place_pub = self.create_publisher(String, "/place/state", 10)
        self.depth, self.depth_t, self.K, self.look_t0 = None, 0.0, None, 0.0
        self.place, self.eval_lock = None, threading.Lock()
        threading.Thread(target=self.place_heartbeat, daemon=True).start()
        self.get_logger().info("pick_place ready. Objects: " + ", ".join(OBJECTS))

    # ---------- helpers ----------
    def say(self, s):
        self.get_logger().info(s)
        self.state_pub.publish(String(data=s))

    def wait(self, fut, timeout=60.0):
        t = time.time()
        while not fut.done():
            if self.abort.is_set():
                raise Abort()
            if time.time() - t > timeout:
                raise RuntimeError("timeout")
            time.sleep(0.02)
        return fut.result()

    def run_action(self, client, goal, timeout=60.0):
        h = self.wait(client.send_goal_async(goal), 10.0)
        if not h.accepted:
            raise RuntimeError("goal rejected")
        self.cur = h
        res = self.wait(h.get_result_async(), timeout).result
        self.cur = None
        return res

    def pose_of(self, x, y, z_tcp):
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z_tcp + TCP_OFFSET
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = QUAT
        return p

    def plan_and_go(self, constraints, what):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints = [constraints]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)
        r = self.run_action(self.mg, goal)
        if r.error_code.val != 1:
            raise RuntimeError(f"{what}: planning/execution failed (error_code={r.error_code.val})")

    def move_joints(self, q, what="ready"):
        self.plan_and_go(Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for j, v in zip(JOINTS, q)]), what)

    def move_pose(self, x, y, z_tcp, what="pose"):
        p = self.pose_of(x, y, z_tcp)
        pc = PositionConstraint(link_name=TOOL, weight=1.0)
        pc.header.frame_id = FRAME
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])], primitive_poses=[p])
        oc = OrientationConstraint(link_name=TOOL, orientation=p.orientation, weight=1.0,
                                   absolute_x_axis_tolerance=0.05, absolute_y_axis_tolerance=0.05,
                                   absolute_z_axis_tolerance=0.05)
        oc.header.frame_id = FRAME
        self.plan_and_go(Constraints(position_constraints=[pc], orientation_constraints=[oc]), what)

    def line_to(self, x, y, z_tcp, what="line"):
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state.is_diff = True
        req.group_name, req.link_name = GROUP, TOOL
        req.waypoints = [self.pose_of(x, y, z_tcp)]
        req.max_step = 0.005
        req.avoid_collisions = False      # short vertical moves; objects are not in the scene yet
        for f, v in (("jump_threshold", 0.0), ("max_velocity_scaling_factor", 0.2),
                     ("max_acceleration_scaling_factor", 0.2)):
            if hasattr(req, f):
                setattr(req, f, v)
        res = self.wait(self.cart.call_async(req), 15.0)
        if res.fraction < 0.99:
            raise RuntimeError(f"{what}: cartesian path only {res.fraction:.2f} complete")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        r = self.run_action(self.ex, goal)
        if r.error_code.val != 1:
            raise RuntimeError(f"{what}: execution failed (error_code={r.error_code.val})")

    def gripper(self, travel):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=FINGERS, points=[JointTrajectoryPoint(
            positions=[travel] * 2, time_from_start=Duration(sec=1, nanosec=500000000))])
        r = self.run_action(self.grip, goal, 10.0)
        if r.error_code != 0:
            raise RuntimeError(f"gripper failed (error_code={r.error_code})")
        time.sleep(0.5)

    # ---------- vision ----------
    def on_det(self, msg):
        try:
            self.det = json.loads(msg.data)
        except ValueError:
            pass

    def look(self, name):
        self.move_joints(HOME, "look pose")
        time.sleep(1.5)                      # let the arm settle so the camera sees the whole table
        t0 = time.time()
        self.look_t0 = t0
        while time.time() - t0 < 10.0:
            if self.abort.is_set():
                raise Abort()
            if self.det.get("t", 0.0) > t0 and name in self.det.get("objs", {}):
                return self.det["objs"][name]
            time.sleep(0.1)
        raise RuntimeError(f"{name} not seen by the camera")

    # ---------- sequence ----------
    def sequence(self, name, goal=None):
        bx, by = BIN
        self.say(f"running:{name}: looking")
        o = self.look(name)
        x, y, top, width = o["x"], o["y"], o["top_z"], o["wx"]
        if abs(x - bx) < BIN_HALF and abs(y - by) < BIN_HALF:
            raise RuntimeError(f"{name} is already in the bin")
        z = max(TABLE_TOP + 0.026, top - 0.04)                         # grasp height (fingertip frame)
        close = min(0.035, max(0.005, (0.075 - width) / 2 + 0.004))    # finger travel for the measured width
        self.say(f"running:{name}: seen at x={x:.3f} y={y:.3f} top={top:.3f} width={width * 100:.1f}cm")
        tx, ty, drop_z = bx, by, DROP_Z
        if goal is not None:
            self.say(f"running:{name}: checking the place goal x={goal[0]:.3f} y={goal[1]:.3f}")
            objs, depth, K = self.wait_inputs(self.look_t0)
            lines, fails, warns, info = evaluate_goal(goal, name, objs, height_grid(world_points(depth, K)), self.ik_ok)
            for ln in lines:
                self.get_logger().info(ln)
            if fails:
                raise RuntimeError("place goal refused: " + "; ".join(fails))
            tx, ty, drop_z = goal[0], goal[1], info["z_release"]
            self.say(f"running:{name}: place goal ok: {info['what']}, release at fingertip z={drop_z:.3f}")
        self.say(f"running:{name}: to ready, open gripper")
        self.move_joints(READY)
        self.gripper(0.0)
        self.say(f"running:{name}: approach")
        self.move_pose(x, y, z + APPROACH, "pre-grasp")
        self.line_to(x, y, z, "descend")
        self.say(f"running:{name}: grasp")
        self.gripper(close)
        self.line_to(x, y, z + LIFT, "lift")
        self.say(f"running:{name}: carry")
        try:
            self.line_to(x, y, TRANSIT_Z, "rise")
            self.line_to(tx, ty, TRANSIT_Z, "transit")
        except RuntimeError as e:
            self.get_logger().warn(f"straight transit failed ({e}); using normal planning")
            self.move_pose(tx, ty, TRANSIT_Z, "above-bin" if goal is None else "above-goal")
        self.line_to(tx, ty, drop_z, "lower into bin" if goal is None else "lower to goal")
        self.say(f"running:{name}: release")
        self.gripper(0.0)
        self.line_to(tx, ty, TRANSIT_Z, "rise from bin" if goal is None else "rise from goal")
        self.move_joints(HOME, "home")

    def worker(self, name, goal=None):
        self.abort.clear()
        try:
            self.sequence(name, goal)
            self.say(f"done:{name}")
        except Abort:
            self.say(f"aborted:{name}")
        except Exception as e:
            self.say(f"failed:{name}: {e}")
        finally:
            self.busy, self.cur = False, None
            self.place = None
            self.publish_place()

    def on_cmd(self, msg):
        try:
            name, goal = parse_cmd(msg.data)
        except (ValueError, KeyError, TypeError, AttributeError) as e:
            self.get_logger().warn(f"bad /pick_place_command {msg.data!r}: {e!r}")
            return
        if name not in OBJECTS:
            self.get_logger().warn(f"unknown object '{name}'")
        elif self.busy:
            self.get_logger().warn("busy; ignoring " + name)
        else:
            self.busy = True
            threading.Thread(target=self.worker, args=(name, goal), daemon=True).start()

    # ---------- place goal ----------
    def on_depth(self, m):
        self.depth = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()
        self.depth_t = time.time()

    def on_info(self, m):
        self.K = list(m.k)

    def ik_ok(self, x, y, z_tcp):
        """Collision-checked IK, tool pointing down, 3 seeds around the target bearing (same method as goto_point.py).
        True / False, or None if the IK service is not ready or does not answer."""
        if not self.ik_cli.service_is_ready():
            return None
        base = math.atan2(y, x) - math.asin(min(0.138 / max(math.hypot(x, y), 0.14), 1.0))
        for pan in (base, base - 0.35, base + 0.35):
            req = GetPositionIK.Request()
            r = req.ik_request
            r.group_name, r.ik_link_name, r.avoid_collisions = GROUP, TOOL, True
            r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=[pan] + REST))
            r.timeout = Duration(sec=0, nanosec=150_000_000)
            r.pose_stamped.header.frame_id = FRAME
            r.pose_stamped.pose = self.pose_of(x, y, z_tcp)
            fut = self.ik_cli.call_async(req)
            t = time.time()
            while not fut.done():
                if time.time() - t > 3.0:
                    return None
                time.sleep(0.02)
            if fut.result().error_code.val == 1:
                return True
        return False

    def inputs(self, newer_than=0.0):
        """(objs, depth, K, '') if detections and the depth image are fresh, else (None, None, None, reason)."""
        det = self.det or {}
        objs = det.get("objs", {})
        if not objs or time.time() - det.get("t", 0.0) > 3.0:
            return None, None, None, "no fresh /detections (object_detector running? arm at home?)"
        if self.depth is None or self.K is None or time.time() - self.depth_t > 3.0 or self.depth_t < newer_than:
            return None, None, None, "no fresh depth image"
        return objs, self.depth, self.K, ""

    def wait_inputs(self, newer_than):
        t, why = time.time(), ""
        while time.time() - t < 8.0:
            if self.abort.is_set():
                raise Abort()
            objs, depth, K, why = self.inputs(newer_than)
            if not why:
                return objs, depth, K
            time.sleep(0.1)
        raise RuntimeError(why)

    def evaluate_click(self, name, u, v):
        res = {"te": time.time(), "object": name, "goal": None, "ok": False, "fails": [], "warns": [], "lines": []}
        if name not in OBJECTS:
            res["fails"].append("unknown object")
            return res
        if self.busy:
            res["fails"].append("robot busy")
            return res
        objs, depth, K, why = self.inputs()
        if why:
            res["fails"].append(why)
            return res
        if name not in objs:
            res["fails"].append(f"{name} is not seen (arm at home?)")
            return res
        pt = click_to_world(u, v, depth, K)
        if pt is None:
            res["fails"].append("no valid depth at that pixel")
            return res
        snap = snap_goal(pt, objs, name)
        gx, gy, mu, mv = (snap[0], snap[1], snap[2][0], snap[2][1]) if snap else (pt[0], pt[1], u, v)
        res["goal"] = {"x": round(float(gx), 3), "y": round(float(gy), 3), "u": round(float(mu), 1), "v": round(float(mv), 1),
                       "snap": snap[3] if snap else None}
        lines, fails, warns, info = evaluate_goal((gx, gy), name, objs, height_grid(world_points(depth, K)), self.ik_ok)
        res.update(lines=lines, fails=fails, warns=warns, ok=not fails)
        for k in ("what", "support", "z_release", "frac"):
            if k in info:
                res[k] = info[k] if k == "what" else round(float(info[k]), 3)
        return res

    def publish_place(self):
        p = self.place
        d = {"t": time.time(), "te": 0.0, "object": None, "goal": None, "ok": False, "pending": False, "fails": [], "warns": []}
        if p:
            for k in ("te", "object", "goal", "ok", "pending", "fails", "warns", "what", "support", "z_release", "frac"):
                if k in p:
                    d[k] = p[k]
        self.place_pub.publish(String(data=json.dumps(d)))

    def place_heartbeat(self):
        while True:
            time.sleep(1.0)
            try:
                self.publish_place()
            except Exception:
                pass

    def on_place_click(self, msg):
        threading.Thread(target=self.eval_click, args=(msg.data,), daemon=True).start()

    def eval_click(self, raw):
        with self.eval_lock:
            try:
                c = json.loads(raw)
                name, u, v = str(c["object"]).strip().lower(), float(c["u"]), float(c["v"])
            except (ValueError, KeyError, TypeError):
                self.get_logger().warn(f"bad /place/click message: {raw!r}")
                return
            self.place = {"te": time.time(), "object": name, "goal": None, "ok": False, "pending": True, "fails": [], "warns": []}
            self.publish_place()
            try:
                self.place = self.evaluate_click(name, u, v)
            except Exception as e:
                self.place = {"te": time.time(), "object": name, "goal": None, "ok": False, "fails": [f"check failed: {e!r}"], "warns": []}
            if self.place.get("lines"):
                for ln in self.place["lines"]:
                    self.get_logger().info(ln)
            else:
                self.get_logger().info(f"place goal for {name}: " + "; ".join(self.place["fails"]))
            self.publish_place()

    def on_place_clear(self, msg):
        self.place = None
        self.publish_place()

    def on_arm_cmd(self, msg):
        if msg.data.strip().lower() == "stop" and self.busy:
            self.abort.set()
            if self.cur is not None:
                self.cur.cancel_goal_async()
            self.get_logger().warn("STOP: aborting pick and place")


def main():
    rclpy.init()
    node = PickPlace()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()


if __name__ == "__main__":
    main()
