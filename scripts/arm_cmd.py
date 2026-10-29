#!/usr/bin/env python3
"""arm_cmd.py: publish ONE /arm_command message, but only after the required subscriber nodes are matched.
Replaces 'ros2 topic pub --once', which can publish before every subscriber has matched and so lose the message.
usage: arm_cmd.py COMMAND [REQUIRED_NODE ...]    (default required node: arm_control)
Runs INSIDE the sim container (arm_cmd.sh calls it through run.sh exec)."""
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "/arm_command"
SETTLE = 1.0   # s: the list of subscribers must stay unchanged this long before sending (late-discovered nodes still get the message)


def main():
    if len(sys.argv) < 2:
        print("usage: arm_cmd.py COMMAND [REQUIRED_NODE ...]")
        sys.exit(2)
    cmd, need = sys.argv[1], sys.argv[2:] or ["arm_control"]
    rclpy.init()
    n = Node("arm_cmd_cli")
    pub = n.create_publisher(String, TOPIC, 10)
    t0, names, infos = time.time(), [], []
    last_sig, stable_since = None, time.time()
    while time.time() - t0 < 10.0:
        infos = n.get_subscriptions_info_by_topic(TOPIC)
        names = [i.node_name for i in infos]
        sig = sorted(names)
        if sig != last_sig:
            last_sig, stable_since = sig, time.time()
        if (all(r in names for r in need) and pub.get_subscription_count() >= len(infos)
                and time.time() - stable_since >= SETTLE):
            break
        rclpy.spin_once(n, timeout_sec=0.1)
    missing = [r for r in need if r not in names]
    if missing or pub.get_subscription_count() < len(infos):
        print("NOT SENT: required %s, subscribers seen %s, matched %d of %d"
              % (need, sorted(set(names)), pub.get_subscription_count(), len(infos)))
        sys.exit(1)
    time.sleep(0.3)
    pub.publish(String(data=cmd))
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)
    print("sent %s '%s' | matched subscribers: %s" % (TOPIC, cmd, sorted(set(names))))
    n.destroy_node()
    rclpy.shutdown()


main()
