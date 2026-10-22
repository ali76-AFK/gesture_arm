#!/usr/bin/env python3
"""gesture_interpreter: /gesture events -> /arm_command and /pick_place_command, with safety rules."""
import json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LEFT_MAP = {"fist": "close", "three_fingers": "open", "victory": "home"}
COOLDOWN = 1.5        # s: the same command is ignored if repeated sooner
MAX_AGE = 1.0         # s: older gesture events are dropped
SELECT_TTL = 30.0     # s: a selected object expires after this
PICK_STALE = 60.0     # s: a pick with no state message for this long is assumed finished


class Interpreter(Node):
    def __init__(self):
        super().__init__("gesture_interpreter")
        self.declare_parameter("dry_run", False)
        self.dry = bool(self.get_parameter("dry_run").value)
        self.arm_pub = self.create_publisher(String, "/arm_command", 10)
        self.pick_pub = self.create_publisher(String, "/pick_place_command", 10)
        self.create_subscription(String, "/gesture", self.on_gesture, 10)
        self.create_subscription(String, "/arm_state", self.on_arm_state, 10)
        self.create_subscription(String, "/pick_place_state", self.on_pick_state, 10)
        self.create_subscription(String, "/selected_object", self.on_selected, 10)
        self.arm_state, self.pick_state, self.pick_t = "", "", 0.0
        self.selected, self.selected_t = None, 0.0
        self.last_sent = {}
        self.get_logger().info(f"gesture_interpreter ready (dry_run={self.dry})")

    def on_arm_state(self, m): self.arm_state = m.data

    def on_pick_state(self, m):
        self.pick_state, self.pick_t = m.data, time.time()

    def on_selected(self, m):
        self.selected, self.selected_t = m.data.strip().lower() or None, time.time()
        self.get_logger().info(f"selected object: {self.selected}")

    def busy(self):
        if self.arm_state.startswith("moving:"):
            return f"arm {self.arm_state}"
        if self.pick_state.startswith("running:") and time.time() - self.pick_t < PICK_STALE:
            return f"pick {self.pick_state}"
        return ""

    def send(self, pub, name, text):
        if self.dry:
            self.get_logger().info(f"DRY RUN: would send {name} '{text}'")
        else:
            pub.publish(String(data=text))
            self.get_logger().info(f"sent {name} '{text}'")

    def repeated(self, key):
        now = time.time()
        if now - self.last_sent.get(key, 0.0) < COOLDOWN:
            self.get_logger().info(f"ignored '{key}': repeated within {COOLDOWN}s")
            return True
        self.last_sent[key] = now
        return False

    def on_gesture(self, msg):
        try:
            e = json.loads(msg.data); side, g, t = e["side"], e["gesture"], float(e["t"])
        except (ValueError, KeyError):
            return
        if time.time() - t > MAX_AGE:
            self.get_logger().info(f"dropped stale event {side} {g}")
            return
        if g == "stop":
            self.selected = None
            self.send(self.arm_pub, "/arm_command", "stop")
            return
        if side == "Right":
            self.get_logger().info(f"right hand {g}: not wired yet (needs the dashboard)")
            return
        busy = self.busy()
        if g in LEFT_MAP:
            if busy:
                self.get_logger().info(f"ignored left {g}: {busy}")
            elif not self.repeated(LEFT_MAP[g]):
                self.send(self.arm_pub, "/arm_command", LEFT_MAP[g])
        elif g == "thumbs_up":
            fresh = self.selected and time.time() - self.selected_t < SELECT_TTL
            if busy:
                self.get_logger().info(f"ignored confirm: {busy}")
            elif not fresh:
                self.get_logger().info("ignored confirm: no object selected in the last 30 s")
            elif not self.repeated("confirm"):
                self.send(self.pick_pub, "/pick_place_command", self.selected)
                self.selected = None
        elif g == "thumbs_down":
            self.selected = None
            self.get_logger().info("cancel: selection cleared")
            if busy:
                self.send(self.arm_pub, "/arm_command", "stop")
        else:
            self.get_logger().info(f"left {g}: no command")


def main():
    rclpy.init()
    rclpy.spin(Interpreter())


if __name__ == "__main__":
    main()
