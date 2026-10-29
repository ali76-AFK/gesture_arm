#!/usr/bin/env python3
"""hand_state_probe (READ-ONLY, no motion): listens to /hand_state for N seconds (default 15) and prints the message
rate, the largest gap between messages, and per hand and label: message count, confidence range, longest unbroken run.
usage: hand_state_probe.py [SECONDS]      Show your left thumbs up (hold it ~8 s) while it runs.
Runs INSIDE the sim container (via run.sh exec)."""
import json
import sys
import time

RUN_GAP = 0.5   # s: two messages further apart than this do not belong to the same run


def summarize(msgs):
    """msgs: list of (receive_time_s, dict). Returns the text report."""
    if not msgs:
        return "no /hand_state message received (no hand visible, or the gesture node is not running)"
    ts = [t for t, _ in msgs]
    span = ts[-1] - ts[0]
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    out = ["messages: %d over %.1f s between first and last (%.1f Hz) | largest gap %.2f s"
           % (len(msgs), span, (len(msgs) - 1) / span if span > 0 else 0.0, max(gaps) if gaps else 0.0)]
    stats, cur, runs = {}, {}, {}
    for t, m in msgs:
        for hand, h in (m.get("hands") or {}).items():
            key = (hand, h.get("label"))
            s = stats.setdefault(key, {"n": 0, "lo": 9.0, "hi": -9.0})
            s["n"] += 1
            c = float(h.get("conf", 0.0))
            s["lo"], s["hi"] = min(s["lo"], c), max(s["hi"], c)
            c0 = cur.get(hand)
            if c0 and c0[0] == h.get("label") and t - c0[2] <= RUN_GAP:
                cur[hand] = (c0[0], c0[1], t)
            else:
                cur[hand] = (h.get("label"), t, t)
            lab, t0, t1 = cur[hand]
            runs[(hand, lab)] = max(runs.get((hand, lab), 0.0), t1 - t0)
    for (hand, lab), s in sorted(stats.items(), key=lambda kv: (str(kv[0][0]), str(kv[0][1]))):
        out.append("%s %-14s n=%-4d conf %.2f-%.2f | longest unbroken run %.1f s"
                   % (hand, lab, s["n"], s["lo"], s["hi"], runs.get((hand, lab), 0.0)))
    return "\n".join(out)


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    n = Node("hand_state_probe")
    msgs = []

    def cb(m):
        try:
            msgs.append((time.time(), json.loads(m.data)))
        except ValueError:
            pass
    n.create_subscription(String, "/hand_state", cb, 50)
    print("listening to /hand_state for %.0f s: show your left thumbs up now ..." % secs)
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.05)
    print(summarize(msgs))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
