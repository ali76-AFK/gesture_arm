#!/usr/bin/env python3
"""goto_cli: send clicks to goto_point and print the state it publishes.
usage: goto_cli.py click U V | worldclick X Y [SURFACE_Z] | go | clear | stop | state"""
import json
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

FX, CX, CY = 554.4, 320.0, 240.0
CAM = (0.6, 0.0, 1.25)


def spin(n, sec):
    t = time.time()
    while time.time() - t < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def show(state):
    if state is None:
        print("no /goto/state received (is goto_point running?)")
        return
    print(f"status: {state['status']} | moving: {state['moving']}")
    for w in state["waypoints"]:
        print(f"   waypoint {w['n']}: pixel ({w['u']},{w['v']}) -> x={w['x']} y={w['y']} surface z={w['z_surface']} "
              f"fingertip target z={w['z_tcp']} reachable={w['reachable']}")
    if not state["waypoints"]:
        print("   (no waypoints)")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(1)
    cmd = sys.argv[1]
    rclpy.init()
    n = Node("goto_cli")
    pubs = {k: n.create_publisher(String, t, 10) for k, t in
            (("click", "/goto/click"), ("go", "/goto/go"), ("clear", "/goto/clear"), ("stop", "/arm_command"))}
    box = {}
    n.create_subscription(String, "/goto/state", lambda m: box.update(s=json.loads(m.data)), 10)
    t0 = time.time()
    while pubs["go"].get_subscription_count() == 0 and time.time() - t0 < 10:
        spin(n, 0.2)
    if pubs["go"].get_subscription_count() == 0:
        print("goto_point is not listening on /goto/go")
        raise SystemExit(1)
    if cmd == "click":
        pubs["click"].publish(String(data=json.dumps({"u": float(sys.argv[2]), "v": float(sys.argv[3])})))
        spin(n, 3.5)
    elif cmd == "worldclick":
        x, y = float(sys.argv[2]), float(sys.argv[3])
        zs = float(sys.argv[4]) if len(sys.argv) > 4 else 0.20
        d = CAM[2] - zs
        u, v = CX - (y - CAM[1]) * FX / d, CY - (x - CAM[0]) * FX / d
        print(f"clicking world ({x:.3f}, {y:.3f}) on a surface at z={zs:.2f} -> pixel ({u:.1f}, {v:.1f})")
        pubs["click"].publish(String(data=json.dumps({"u": u, "v": v})))
        spin(n, 3.5)
    elif cmd == "go":
        pubs["go"].publish(String(data="go"))
        last, t1 = None, time.time()
        while time.time() - t1 < 120:
            spin(n, 0.3)
            s = box.get("s")
            if s and s["status"] != last:
                last = s["status"]
                print(f"   [{time.time() - t1:5.1f}s] {last}")
            if s and time.time() - t1 > 2 and not s["moving"]:
                break
    elif cmd == "clear":
        pubs["clear"].publish(String(data="clear"))
        spin(n, 1.5)
    elif cmd == "stop":
        pubs["stop"].publish(String(data="stop"))
        spin(n, 1.5)
    else:
        spin(n, 2.0)
    show(box.get("s"))


if __name__ == "__main__":
    main()
