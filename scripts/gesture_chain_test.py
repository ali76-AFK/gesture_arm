import json, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

rclpy.init(); n = Node("gesture_chain_test")
pub_cmd = n.create_publisher(String, "/arm_command", 10)
pub_g = n.create_publisher(String, "/gesture", 10)
t0 = time.time(); log = []; st = {"s": "", "elbow": None}
T = lambda: time.time() - t0
def on_state(m):
    if m.data != st["s"]: st["s"] = m.data; log.append((T(), "arm_state", m.data))
n.create_subscription(String, "/arm_state", on_state, 10)
n.create_subscription(String, "/arm_command", lambda m: log.append((T(), "arm_command", m.data)), 10)
def on_js(m):
    if "elbow_joint" in m.name: st["elbow"] = m.position[m.name.index("elbow_joint")]
n.create_subscription(JointState, "/joint_states", on_js, 10)

def spin(sec):
    e = time.time() + sec
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.02)
def wait_done(timeout=30):
    spin(0.6); e = time.time() + timeout
    while time.time() < e:
        rclpy.spin_once(n, timeout_sec=0.05)
        if st["s"].startswith(("done:", "failed:")): return st["s"]
    return "timeout"
def gesture(side, g, age=0.0):
    pub_g.publish(String(data=json.dumps({"t": time.time() - age, "side": side, "gesture": g, "conf": 1.0})))
def cmd(c): pub_cmd.publish(String(data=c))
def fmt(v): return "n/a" if v is None else f"{v:.3f}"
def since(mark): return [f"+{t - mark:4.1f}s {k} {v}" for t, k, v in log if t >= mark]

while pub_g.get_subscription_count() == 0 or pub_cmd.get_subscription_count() == 0: spin(0.2)
spin(1.0)

print("reset: ready ->", (cmd("ready"), wait_done())[1], "| elbow", fmt(st["elbow"]))
print("\n== TEST A: victory, then STOP 0.8 s later"); mark = T()
gesture("Left", "victory"); spin(0.8)
e0, s0 = st["elbow"], st["s"]; gesture("Right", "stop"); spin(0.3); e1 = st["elbow"]; spin(1.8); e2 = st["elbow"]
print(f"   arm_state when stop was sent: {s0} | elbow at stop {fmt(e0)}, +0.3 s {fmt(e1)}, +2.1 s {fmt(e2)}   (ready 1.300, home 0.000)")
print("   events:", since(mark))

print("\nreset: ready ->", (cmd("ready"), wait_done())[1]); spin(2.0)
print("\n== TEST B: victory, then LEFT FIST 0.8 s later (fist should be ignored)"); mark = T()
gesture("Left", "victory"); spin(0.8); gesture("Left", "fist"); print("   final:", wait_done())
print("   events:", since(mark))

spin(2.0)
print("\n== TEST D: stale event (stamped 3 s ago): left fist"); mark = T()
gesture("Left", "fist", age=3.0); spin(1.5); print("   events:", since(mark) or "none (expected: dropped)")

print("\n== TEST E: two left fists 0.5 s apart while idle"); mark = T()
gesture("Left", "fist"); spin(0.5); gesture("Left", "fist"); spin(3.0); print("   events:", since(mark))
cmd("open"); spin(2.0)
print("\ndone")
