#!/usr/bin/env python3
"""goto_point (stage 1, backend only): clicked camera pixels -> 3D points from the depth image -> hover waypoints for the
gripper (tool pointing down) -> MoveIt pose goals. It never touches the gripper and never descends.
Topics  in : /goto/click (String JSON {"u":..,"v":..}), /goto/go, /goto/clear (String, any data), /arm_command ('stop' aborts)
        out: /goto/state (String JSON: status and waypoints)"""
import json
import math
import queue
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MotionPlanRequest, OrientationConstraint,
                             JointConstraint, PlanningOptions, PositionConstraint, RobotState)
from moveit_msgs.srv import GetPositionIK

CAM = (0.6, 0.0, 1.25)                    # camera position in base_link, looking straight down (as in object_detector.py)
QUAT = (0.7071068, -0.7071068, 0.0, 0.0)  # x, y, z, w: tool pointing down, same as pick_place.py
TCP_OFFSET = 0.115                        # tool0 -> fingertip frame
GROUP, FRAME, TOOL = "ur_manipulator", "base_link", "tool0"
HOVER, MIN_TCP_Z, MAX_WAYPOINTS = 0.10, 0.35, 2     # fingertip hovers 10 cm above the surface, never below 0.35 m
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
REST = [-1.4, 1.3, -1.47, -1.5708, 0.0]   # lift, elbow, wrist 1, 2, 3 of the 'ready' pose (IK seeds)
PICK_STALE = 60.0


class Abort(Exception):
    pass


def pixel_to_world(u, v, depth, K, cam=CAM, half=4):
    """Pixel (u right, v down) -> base_link point, using the nearest surface in a (2*half+1)^2 window so hovering clears object edges."""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    h, w = depth.shape
    iu, iv = int(round(u)), int(round(v))
    if not (0 <= iu < w and 0 <= iv < h):
        return None
    win = depth[max(0, iv - half):iv + half + 1, max(0, iu - half):iu + half + 1]
    win = win[np.isfinite(win) & (win > 0.05)]
    if win.size < 10:
        return None
    d = float(np.percentile(win, 10))
    return (cam[0] - (v - cy) * d / fy, cam[1] - (u - cx) * d / fx, cam[2] - d)


class GotoPoint(Node):
    def __init__(self):
        super().__init__("goto_point")
        self.q = queue.Queue()
        cb = ReentrantCallbackGroup()
        self.mg = ActionClient(self, MoveGroup, "/move_action", callback_group=cb)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik", callback_group=cb)
        self.depth, self.K = None, None
        self.waypoints, self.status = [], "idle"
        self.abort, self.cur, self.moving = threading.Event(), None, False
        self.arm_state, self.pick_state, self.pick_t = "", "", 0.0
        self.state_pub = self.create_publisher(String, "/goto/state", 10)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, 1, callback_group=cb)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 1, callback_group=cb)
        self.create_subscription(String, "/goto/click", lambda m: self.q.put(("click", m.data)), 10, callback_group=cb)
        self.create_subscription(String, "/goto/go", lambda m: self.q.put(("go", "")), 10, callback_group=cb)
        self.create_subscription(String, "/goto/clear", lambda m: self.q.put(("clear", "")), 10, callback_group=cb)
        self.create_subscription(String, "/arm_command", self.on_arm_cmd, 10, callback_group=cb)
        self.create_subscription(String, "/arm_state", self.on_arm_state, 10, callback_group=cb)
        self.create_subscription(String, "/pick_place_state", self.on_pick_state, 10, callback_group=cb)
        threading.Thread(target=self.worker, daemon=True).start()
        threading.Thread(target=self.heartbeat, daemon=True).start()
        self.get_logger().info(f"goto_point ready: hover {HOVER} m above the surface, fingertip never below {MIN_TCP_Z} m, up to {MAX_WAYPOINTS} waypoints")

    # ---------- inputs ----------
    def on_depth(self, m):
        self.depth = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()

    def on_info(self, m):
        self.K = list(m.k)

    def on_arm_state(self, m):
        self.arm_state = m.data

    def on_pick_state(self, m):
        self.pick_state, self.pick_t = m.data, time.time()

    def on_arm_cmd(self, m):
        if m.data.strip().lower() == "stop" and self.moving:
            self.abort.set()
            if self.cur is not None:
                self.cur.cancel_goal_async()
            self.get_logger().warn("STOP: aborting go-to")

    # ---------- state ----------
    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps(
            {"t": time.time(), "status": self.status, "moving": self.moving, "waypoints": self.waypoints})))

    def say(self, text):
        self.get_logger().info(text)
        self.status = text if not self.moving else self.status
        self.publish_state()

    def heartbeat(self):
        while True:
            time.sleep(1.0)
            self.publish_state()

    def robot_busy(self):
        if self.arm_state.startswith("moving:"):
            return f"arm {self.arm_state}"
        if self.pick_state.startswith("running:") and time.time() - self.pick_t < PICK_STALE:
            return f"pick {self.pick_state}"
        return ""

    # ---------- helpers ----------
    def wait(self, fut, timeout):
        t = time.time()
        while not fut.done():
            if self.abort.is_set():
                raise Abort()
            if time.time() - t > timeout:
                raise RuntimeError("timeout")
            time.sleep(0.02)
        return fut.result()

    def call(self, cli, req, timeout=3.0):
        fut = cli.call_async(req)
        t = time.time()
        while not fut.done():
            if time.time() - t > timeout:
                return None
            time.sleep(0.02)
        return fut.result()

    def solve(self, x, y, z_tcp):
        """Collision-checked IK with three seeds around the target bearing (same method as scripts/reach_map.py).
        Returns the 6 joint values of the first solution, [] if there is none, None if the IK service is not ready."""
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
            r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y = x, y
            r.pose_stamped.pose.position.z = z_tcp + TCP_OFFSET
            (r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y,
             r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w) = QUAT
            res = self.call(self.ik_cli, req)
            if res is not None and res.error_code.val == 1:
                pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [float(pos[j]) for j in JOINTS]
        return []

    def reachable(self, x, y, z_tcp):
        sol = self.solve(x, y, z_tcp)
        return None if sol is None else bool(sol)

    def move_to(self, x, y, z_tcp):
        sol = self.solve(x, y, z_tcp)
        if sol is None:
            raise RuntimeError("IK service not ready")
        if not sol:
            raise RuntimeError("no collision-free IK solution at go time")
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        jc = [JointConstraint(joint_name=n, position=float(q), tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
              for n, q in zip(JOINTS, sol)]
        req.goal_constraints = [Constraints(joint_constraints=jc)]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)
        h = self.wait(self.mg.send_goal_async(goal), 10.0)
        if not h.accepted:
            raise RuntimeError("goal rejected")
        self.cur = h
        res = self.wait(h.get_result_async(), 90.0).result
        self.cur = None
        if res.error_code.val != 1:
            raise RuntimeError(f"planning or execution failed (error_code={res.error_code.val})")

    # ---------- worker: one event at a time ----------
    def worker(self):
        while True:
            kind, data = self.q.get()
            try:
                if kind == "click":
                    self.do_click(data)
                elif kind == "go":
                    self.do_go()
                elif kind == "clear":
                    self.do_clear()
            except Exception as e:      # keep the node alive
                self.get_logger().error(f"{kind} failed: {e!r}")

    def do_clear(self):
        if self.moving:
            self.say("clear ignored: moving (use stop)")
            return
        self.waypoints, self.status = [], "idle"
        self.publish_state()

    def do_click(self, raw):
        try:
            c = json.loads(raw)
            u, v = float(c["u"]), float(c["v"])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f"bad click message: {raw!r}")
            return
        if self.moving:
            self.say("click ignored: moving")
            return
        if self.status.startswith(("done", "aborted", "failed")):
            self.waypoints = []
        if len(self.waypoints) >= MAX_WAYPOINTS:
            self.say(f"click ignored: {MAX_WAYPOINTS} waypoints already set (clear them first)")
            return
        if self.depth is None or self.K is None:
            self.say("click ignored: no depth image yet")
            return
        pt = pixel_to_world(u, v, self.depth, self.K)
        if pt is None:
            self.say("click ignored: no valid depth at that pixel")
            return
        x, y, zs = pt
        z_tcp = max(zs + HOVER, MIN_TCP_Z)
        ok = self.reachable(x, y, z_tcp)
        self.waypoints.append({"n": len(self.waypoints) + 1, "u": round(u, 1), "v": round(v, 1), "x": round(x, 3),
                               "y": round(y, 3), "z_surface": round(zs, 3), "z_tcp": round(z_tcp, 3), "reachable": ok})
        self.status = "idle"
        self.get_logger().info(f"waypoint {len(self.waypoints)}: pixel ({u:.0f},{v:.0f}) -> x={x:.3f} y={y:.3f} surface z={zs:.3f}, "
                               f"fingertip target z={z_tcp:.3f}, reachable={ok}")
        self.publish_state()

    def do_go(self):
        if self.moving:
            return
        if not self.waypoints:
            self.say("go ignored: no waypoints")
            return
        bad = [w for w in self.waypoints if w["reachable"] is not True]
        if bad:
            self.say(f"not moving: waypoint {bad[0]['n']} is not reachable")
            return
        busy = self.robot_busy()
        if busy:
            self.say(f"not moving: robot busy ({busy})")
            return
        self.abort.clear()
        self.moving = True
        try:
            for w in list(self.waypoints):
                if self.abort.is_set():
                    raise Abort()
                self.status = f"moving:{w['n']}"
                self.publish_state()
                self.move_to(w["x"], w["y"], w["z_tcp"])
            self.status = "done"
        except Abort:
            self.status = "aborted"
        except Exception as e:
            self.status = f"failed: {e}"
            self.get_logger().error(self.status)
        finally:
            self.moving, self.cur = False, None
            self.publish_state()
        self.get_logger().info(f"go-to finished: {self.status}")


def main():
    rclpy.init()
    node = GotoPoint()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()


if __name__ == "__main__":
    main()
