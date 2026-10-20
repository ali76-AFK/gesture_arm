#!/usr/bin/env python3
"""pick_place: hard-coded objects -> top-down pick, carry, drop into the bin."""
import json, threading, time
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
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint, PlanningOptions,
                             PositionConstraint, OrientationConstraint, BoundingVolume)

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
        while time.time() - t0 < 10.0:
            if self.abort.is_set():
                raise Abort()
            if self.det.get("t", 0.0) > t0 and name in self.det.get("objs", {}):
                return self.det["objs"][name]
            time.sleep(0.1)
        raise RuntimeError(f"{name} not seen by the camera")

    # ---------- sequence ----------
    def sequence(self, name):
        bx, by = BIN
        self.say(f"running:{name}: looking")
        o = self.look(name)
        x, y, top, width = o["x"], o["y"], o["top_z"], o["wx"]
        if abs(x - bx) < BIN_HALF and abs(y - by) < BIN_HALF:
            raise RuntimeError(f"{name} is already in the bin")
        z = max(TABLE_TOP + 0.026, top - 0.04)                         # grasp height (fingertip frame)
        close = min(0.035, max(0.005, (0.075 - width) / 2 + 0.004))    # finger travel for the measured width
        self.say(f"running:{name}: seen at x={x:.3f} y={y:.3f} top={top:.3f} width={width * 100:.1f}cm")
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
            self.line_to(bx, by, TRANSIT_Z, "transit")
        except RuntimeError as e:
            self.get_logger().warn(f"straight transit failed ({e}); using normal planning")
            self.move_pose(bx, by, TRANSIT_Z, "above-bin")
        self.line_to(bx, by, DROP_Z, "lower into bin")
        self.say(f"running:{name}: release")
        self.gripper(0.0)
        self.line_to(bx, by, TRANSIT_Z, "rise from bin")
        self.move_joints(HOME, "home")

    def worker(self, name):
        self.abort.clear()
        try:
            self.sequence(name)
            self.say(f"done:{name}")
        except Abort:
            self.say(f"aborted:{name}")
        except Exception as e:
            self.say(f"failed:{name}: {e}")
        finally:
            self.busy, self.cur = False, None

    def on_cmd(self, msg):
        name = msg.data.strip().lower()
        if name not in OBJECTS:
            self.get_logger().warn(f"unknown object '{name}'")
        elif self.busy:
            self.get_logger().warn("busy; ignoring " + name)
        else:
            self.busy = True
            threading.Thread(target=self.worker, args=(name,), daemon=True).start()

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
