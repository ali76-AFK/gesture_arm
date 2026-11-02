#!/usr/bin/env python3
"""place_cli: test helper for the place-goal feature, no browser or gestures needed. Runs INSIDE the sim container (run.sh exec).
  state                               print the latest /place/state
  click OBJECT U V                    send a goal click at image pixel (U, V), print the check result
  worldclick OBJECT X Y [SURFACE_Z]   same, with the pixel computed from a world point on a surface at SURFACE_Z (default 0.20 = table)
  clear                               send /place/clear
  objs                                print the latest /detections compactly
  run OBJECT [X Y]                    MOVES THE ARM: sends /pick_place_command (plain name = to the bin, with X Y = to that goal)"""
import json
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

FX, CX, CY, CAM_X, CAM_Z = 554.4, 320.0, 240.0, 0.6, 1.25      # camera model (world file)


def world_to_pixel(x, y, z=0.20):
    d = CAM_Z - z
    return CX - y * FX / d, CY + (CAM_X - x) * FX / d


def fmt(s):
    out = ["object %s | te %.1f | ok %s" % (s.get("object"), s.get("te", 0.0), s.get("ok"))]
    if s.get("goal"):
        g = s["goal"]
        out.append("goal x=%.3f y=%.3f pixel (%.1f, %.1f)%s | %s | support z=%s | release fingertip z=%s | supported fraction %s"
                   % (g["x"], g["y"], g["u"], g["v"], " snapped to " + g["snap"] if g.get("snap") else "", s.get("what"), s.get("support"), s.get("z_release"), s.get("frac")))
    out.append("fails: %s" % (s.get("fails") or "none"))
    out.append("warns: %s" % (s.get("warns") or "none"))
    return "\n".join(out)


def main():
    a = sys.argv[1:]
    if not a or a[0] not in ("state", "click", "worldclick", "clear", "objs", "run"):
        print(__doc__)
        return
    rclpy.init()
    n = Node("place_cli")
    box = {}
    n.create_subscription(String, "/place/state", lambda m: box.update(state=json.loads(m.data)), 1)
    n.create_subscription(String, "/detections", lambda m: box.update(det=json.loads(m.data)), 1)

    def pub(topic, text):
        p = n.create_publisher(String, topic, 1)
        t = time.time()
        while p.get_subscription_count() == 0 and time.time() - t < 5.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        print("sent %s %s | matched subscribers: %d" % (topic, text, p.get_subscription_count()))
        p.publish(String(data=text))

    def spin(sec, until=None):
        t = time.time()
        while time.time() - t < sec and not (until and until()):
            rclpy.spin_once(n, timeout_sec=0.1)

    cmd = a[0]
    if cmd == "state":
        spin(3.0, lambda: "state" in box)
        print(fmt(box["state"]) if "state" in box else "no /place/state (is pick_place running the patched version?)")
    elif cmd in ("click", "worldclick"):
        if cmd == "click":
            name, u, v = a[1], float(a[2]), float(a[3])
        else:
            name, (u, v) = a[1], world_to_pixel(float(a[2]), float(a[3]), float(a[4]) if len(a) > 4 else 0.20)
        t0 = time.time()
        pub("/place/click", json.dumps({"object": name, "u": round(u, 1), "v": round(v, 1)}))
        done = lambda: box.get("state", {}).get("te", 0.0) > t0 and not box["state"].get("pending")
        spin(8.0, done)
        print(fmt(box["state"]) if "state" in box and done() else "no finished /place/state within 8 s")
    elif cmd == "clear":
        pub("/place/clear", "clear")
    elif cmd == "objs":
        spin(3.0, lambda: "det" in box)
        for k, o in sorted((box.get("det") or {}).get("objs", {}).items()):
            print("%-14s x=%.3f y=%.3f top_z=%.3f size %.3f x %.3f" % (k, o["x"], o["y"], o["top_z"], o["wx"], o["wy"]))
        if "det" not in box:
            print("no /detections")
    elif cmd == "run":
        text = json.dumps({"object": a[1], "goal": {"x": float(a[2]), "y": float(a[3])}}) if len(a) >= 4 else a[1]
        pub("/pick_place_command", text)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
