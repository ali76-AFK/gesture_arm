#!/usr/bin/env python3
"""test_goto_stop2 (MOVES the arm). Needs: sim + arm_control + goto_point + pick_place running, arm at home, 2 waypoints set.
usage: test_goto_stop2.py [DELAY_S]   (default 0.7 s after the first 'moving:' state)
Publisher is created and matched BEFORE 'go' is sent; exactly one stop is published; no ros2 CLI start-up delay."""
import json
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    delay = float(sys.argv[1]) if len(sys.argv) > 1 else 0.7
    rclpy.init()
    n = Node("test_goto_stop2")
    box = {}
    n.create_subscription(String, "/goto/state", lambda m: box.update(s=json.loads(m.data)), 10)
    go = n.create_publisher(String, "/goto/go", 10)
    stop = n.create_publisher(String, "/arm_command", 10)

    def spin(sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(n, timeout_sec=0.02)

    t0 = time.time()
    while time.time() - t0 < 10 and (go.get_subscription_count() < 1 or stop.get_subscription_count() < 3 or "s" not in box):
        spin(0.1)
    print("subscribers: /goto/go %d, /arm_command %d (expect 3), state seen: %s"
          % (go.get_subscription_count(), stop.get_subscription_count(), "s" in box))
    if go.get_subscription_count() < 1 or stop.get_subscription_count() < 3 or "s" not in box:
        print("not ready, nothing sent")
        return
    if box["s"]["moving"] or not box["s"]["waypoints"]:
        print("not starting: moving=%s waypoints=%d" % (box["s"]["moving"], len(box["s"]["waypoints"])))
        return
    go.publish(String(data="go"))
    t_go = time.time()
    while time.time() - t_go < 15 and not (box["s"]["moving"] and box["s"]["status"].startswith("moving:")):
        spin(0.02)
    if not (box["s"]["moving"] and box["s"]["status"].startswith("moving:")):
        print("go did not start, status=%r" % box["s"]["status"])
        return
    print("moving seen +%.2f s after go (%s)" % (time.time() - t_go, box["s"]["status"]))
    spin(delay)
    t_stop = time.time()
    stop.publish(String(data="stop"))
    while time.time() - t_stop < 30:
        spin(0.02)
        if not box["s"]["moving"]:
            break
    spin(0.5)
    print("stop sent %.2f s after the first moving state; go-to ended %.2f s after the stop" % (delay, time.time() - t_stop))
    print("final status: %s | moving: %s" % (box["s"]["status"], box["s"]["moving"]))
    n.destroy_node()
    rclpy.shutdown()


main()
