#!/usr/bin/env python3
"""arm_control: named poses + collision scene for the UR5e via MoveIt's move_action."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint,
                             PlanningOptions, PlanningScene, PlanningSceneWorld,
                             CollisionObject)

GROUP = "ur_manipulator"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
NAMED = {
    "home":    [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0],      # arm straight up
    "ready":   [0.0, -1.4, 1.3, -1.47, -1.5708, 0.0],       # tool down, above the table
    "inspect": [0.8, -1.4, 1.3, -1.47, -1.5708, 0.0],       # same, swung to one side
}
TABLE_TOP = 0.20   # must match scripts/make_world.py


def box(x, y, z):
    p = SolidPrimitive(); p.type = SolidPrimitive.BOX; p.dimensions = [x, y, z]; return p


def pose(x, y, z):
    p = Pose(); p.position.x, p.position.y, p.position.z = x, y, z; p.orientation.w = 1.0; return p


def make_scene_objects():
    # frame: base_link (robot base sits at the world origin)
    table = CollisionObject(id="table", operation=CollisionObject.ADD)
    table.header.frame_id = "base_link"
    table.primitives = [box(0.8, 1.0, TABLE_TOP)]
    table.primitive_poses = [pose(0.6, 0.0, TABLE_TOP / 2)]

    cx, cy, w, h, t = 0.55, -0.33, 0.16, 0.06, 0.01
    z0 = TABLE_TOP
    bin_ = CollisionObject(id="bin", operation=CollisionObject.ADD)
    bin_.header.frame_id = "base_link"
    bin_.primitives = [box(w, w, t), box(w, t, h), box(w, t, h), box(t, w - 2 * t, h), box(t, w - 2 * t, h)]
    bin_.primitive_poses = [
        pose(cx, cy, z0 + t / 2),
        pose(cx, cy + w / 2 - t / 2, z0 + t + h / 2),
        pose(cx, cy - w / 2 + t / 2, z0 + t + h / 2),
        pose(cx + w / 2 - t / 2, cy, z0 + t + h / 2),
        pose(cx - w / 2 + t / 2, cy, z0 + t + h / 2),
    ]
    return [table, bin_]


class ArmControl(Node):
    def __init__(self):
        super().__init__("arm_control")
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.state_pub = self.create_publisher(String, "/arm_state", 10)
        self.create_subscription(String, "/arm_command", self.on_command, 10)
        self.busy = False
        self.goal_handle = None
        self.current = ""
        self.set_state("idle")
        self.scene_timer = self.create_timer(2.0, self.add_scene)
        self.create_timer(1.0, lambda: self.state_pub.publish(String(data=self.state)))
        self.get_logger().info("arm_control ready. Commands: " + ", ".join(NAMED) + ", stop")

    def set_state(self, s):
        self.state = s
        self.state_pub.publish(String(data=s))

    # ---- collision scene ----
    def add_scene(self):
        if not self.scene_cli.service_is_ready():
            return
        self.scene_timer.cancel()
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True, world=PlanningSceneWorld(collision_objects=make_scene_objects()))
        self.scene_cli.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(f"collision scene (table, bin) applied: {f.result().success}"))

    # ---- commands ----
    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "stop":
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.get_logger().warn("STOP: cancelling current motion")
            return
        if cmd not in NAMED:
            self.get_logger().warn(f"unknown command '{cmd}'")
            return
        if self.busy:
            self.get_logger().warn(f"busy ({self.state}); ignoring '{cmd}'")
            return
        if not self.client.server_is_ready():
            self.get_logger().error("move_action not available")
            return
        self.move_to(cmd)

    def move_to(self, name):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=p, tolerance_above=0.01,
                            tolerance_below=0.01, weight=1.0)
            for j, p in zip(JOINTS, NAMED[name])])]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)
        self.busy, self.current = True, name
        self.set_state(f"moving:{name}")
        self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        h = fut.result()
        if not h.accepted:
            self.finish(False, "goal rejected")
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        code = fut.result().result.error_code.val
        self.finish(code == 1, f"error_code={code}")

    def finish(self, ok, info):
        self.set_state(("done:" if ok else "failed:") + self.current)
        self.get_logger().info(f"{self.current}: {'OK' if ok else 'FAILED'} ({info})")
        self.busy, self.goal_handle = False, None


def main():
    rclpy.init()
    rclpy.spin(ArmControl())


if __name__ == "__main__":
    main()
