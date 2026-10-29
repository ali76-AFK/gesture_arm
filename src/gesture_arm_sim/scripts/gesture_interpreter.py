#!/usr/bin/env python3
"""gesture_interpreter: /gesture events -> /arm_command and /pick_place_command, with safety rules."""
import json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LEFT_MAP = {"fist": "close", "three_fingers": "open", "victory": "home"}
COOLDOWN = 1.5        # s: the same command is ignored if repeated sooner
MAX_AGE = 1.0         # s: older gesture events are dropped
GOTO_STATE_MAX_AGE = 3.0   # s: a /goto/state older than this counts as missing
GOTO_WP_TTL = 60.0         # s: waypoints unchanged for longer than this are not started by gesture
GOTO_CONFIRM_MIN = 0.8     # s: go-to needs a second thumbs up at least this long after the first
GOTO_CONFIRM_MAX = 5.0     # s: ... and at most this long after
HOLD_LABEL = "thumbs_up"   # /hand_state label of the left hand that confirms a go-to
GOTO_HOLD_SEC = 2.0        # s: left thumbs up must stay on screen this long (after the arming thumbs up) to start a go-to
GOTO_HOLD_CONF = 0.8       # confidence a /hand_state message needs to count towards the hold
GOTO_HOLD_GRACE = 0.4      # s: a break in qualifying messages longer than this restarts the hold
GOTO_HOLD_AFTER_ARM = 1.0 # s: the go-to starts no sooner than this after the arming thumbs up
SELECT_TTL = 30.0     # s: a selected object expires after this
PICK_STALE = 60.0     # s: a pick with no state message for this long is assumed finished


class Interpreter(Node):
    def __init__(self):
        super().__init__("gesture_interpreter")
        self.declare_parameter("dry_run", False)
        self.dry = bool(self.get_parameter("dry_run").value)
        self.arm_pub = self.create_publisher(String, "/arm_command", 10)
        self.pick_pub = self.create_publisher(String, "/pick_place_command", 10)
        self.goto_go_pub = self.create_publisher(String, "/goto/go", 10)
        self.goto_clear_pub = self.create_publisher(String, "/goto/clear", 10)
        self.create_subscription(String, "/gesture", self.on_gesture, 10)
        self.create_subscription(String, "/arm_state", self.on_arm_state, 10)
        self.create_subscription(String, "/pick_place_state", self.on_pick_state, 10)
        self.create_subscription(String, "/selected_object", self.on_selected, 10)
        self.create_subscription(String, "/goto/state", self.on_goto_state, 10)
        self.create_subscription(String, "/hand_state", self.on_hand_state, 20)
        self.arm_state, self.pick_state, self.pick_t = "", "", 0.0
        self.selected, self.selected_t = None, 0.0
        self.goto, self.goto_t, self.goto_sig, self.goto_wp_t, self.goto_armed_t = None, 0.0, [], 0.0, 0.0
        self.hold_start, self.hold_last, self.hold_fired = None, 0.0, False
        self.last_sent = {}
        self.get_logger().info(f"gesture_interpreter ready (dry_run={self.dry})")

    def on_arm_state(self, m): self.arm_state = m.data

    def on_pick_state(self, m):
        self.pick_state, self.pick_t = m.data, time.time()

    def on_goto_state(self, m):
        try:
            g = json.loads(m.data)
        except ValueError:
            return
        self.goto, self.goto_t = g, time.time()
        sig = [(w.get("n"), w.get("u"), w.get("v")) for w in (g.get("waypoints") or [])]
        if sig != self.goto_sig:
            self.goto_sig, self.goto_wp_t = sig, time.time()

    def on_hand_state(self, m):
        try:
            h = (json.loads(m.data).get("hands") or {}).get("Left")
        except (ValueError, AttributeError):
            return
        self.hold_update(h, time.time())

    def hold_update(self, h, now):
        """Tracks how long the LEFT hand has shown HOLD_LABEL without a break (a break = more than GOTO_HOLD_GRACE s
        without a qualifying message). A go-to started by the hold needs the arming thumbs up inside the same run."""
        ok = False
        if h:
            try:
                ok = h.get("label") == HOLD_LABEL and float(h.get("conf", 0.0)) >= GOTO_HOLD_CONF
            except (TypeError, ValueError):
                ok = False
        if ok:
            if self.hold_start is None or now - self.hold_last > GOTO_HOLD_GRACE:
                self.hold_start, self.hold_fired = now, False
            self.hold_last = now
            self.hold_try(now)
        elif self.hold_start is not None and now - self.hold_last > GOTO_HOLD_GRACE:
            self.hold_start = None

    def hold_try(self, now):
        if self.hold_fired or not self.goto_armed_t:
            return
        if self.goto_armed_t < self.hold_start - 0.3 or now - self.goto_armed_t > GOTO_CONFIRM_MAX:
            return
        if self.hold_last - self.hold_start < GOTO_HOLD_SEC or now - self.goto_armed_t < GOTO_HOLD_AFTER_ARM:
            return
        self.hold_fired = True
        self.goto_armed_t = 0.0
        why = self.goto_ready()
        busy = self.busy()
        fresh = self.selected and now - self.selected_t < SELECT_TTL
        if why or busy or fresh:
            self.get_logger().info("hold confirm ignored: " + (why or busy or "an object is selected"))
            return
        if not self.repeated("goto"):
            self.get_logger().info(f"go-to confirmed: left thumbs up held {GOTO_HOLD_SEC:.0f} s")
            self.send(self.goto_go_pub, "/goto/go", "go")

    def goto_ready(self):
        """'' if a go-to may be started by gesture, else the reason it may not."""
        g = self.goto
        if g is None or time.time() - self.goto_t > GOTO_STATE_MAX_AGE:
            return "no fresh /goto/state"
        wps = g.get("waypoints") or []
        if not wps:
            return "no waypoints set"
        if any(w.get("reachable") is not True for w in wps):
            return "a waypoint is not reachable"
        if time.time() - self.goto_wp_t > GOTO_WP_TTL:
            return f"waypoints unchanged for more than {GOTO_WP_TTL:.0f} s"
        return ""

    def goto_confirm(self):
        why = self.goto_ready()
        if why:
            self.get_logger().info(f"ignored confirm: no object selected in the last 30 s; go-to: {why}")
            self.goto_armed_t = 0.0
            return
        now = time.time()
        gap = now - self.goto_armed_t
        if self.goto_armed_t and gap < GOTO_CONFIRM_MIN:
            self.get_logger().info("go-to confirm: second thumbs up too soon, still armed")
        elif self.goto_armed_t and gap <= GOTO_CONFIRM_MAX:
            self.goto_armed_t = 0.0
            if not self.repeated("goto"):
                self.send(self.goto_go_pub, "/goto/go", "go")
        else:
            self.goto_armed_t = now
            self.get_logger().info(f"go-to armed: thumbs up again within {GOTO_CONFIRM_MAX:.0f} s to start")

    def on_selected(self, m):
        self.selected, self.selected_t = m.data.strip().lower() or None, time.time()
        self.get_logger().info(f"selected object: {self.selected}")

    def busy(self):
        if self.arm_state.startswith("moving:"):
            return f"arm {self.arm_state}"
        if self.pick_state.startswith("running:") and time.time() - self.pick_t < PICK_STALE:
            return f"pick {self.pick_state}"
        if self.goto and self.goto.get("moving") and time.time() - self.goto_t < GOTO_STATE_MAX_AGE:
            return "go-to moving"
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
            self.goto_armed_t = 0.0
            self.send(self.arm_pub, "/arm_command", "stop")
            return
        if side == "Right":
            self.get_logger().info(f"right hand {g}: handled by the dashboard, no robot command")
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
                self.goto_confirm()
            elif not self.repeated("confirm"):
                self.send(self.pick_pub, "/pick_place_command", self.selected)
                self.selected = None
        elif g == "thumbs_down":
            self.selected = None
            self.get_logger().info("cancel: selection cleared")
            self.goto_armed_t = 0.0
            if self.goto and not self.goto.get("moving") and self.goto.get("waypoints"):
                self.send(self.goto_clear_pub, "/goto/clear", "clear")
            if busy:
                self.send(self.arm_pub, "/arm_command", "stop")
        else:
            self.get_logger().info(f"left {g}: no command")


def main():
    rclpy.init()
    rclpy.spin(Interpreter())


if __name__ == "__main__":
    main()
