import time, rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init(); n = Node("stop_test")
pub = n.create_publisher(String, "/arm_command", 10)
st = {"s": ""}
n.create_subscription(String, "/arm_state", lambda m: st.update(s=m.data), 10)

def spin(sec):
    t = time.time()
    while time.time() - t < sec: rclpy.spin_once(n, timeout_sec=0.05)

def wait_done(timeout=25):
    spin(0.5)
    t = time.time()
    while time.time() - t < timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        if st["s"].startswith(("done:", "failed:")): return st["s"]
    return "timeout"

while pub.get_subscription_count() == 0: spin(0.2)
pub.publish(String(data="home")); print("sent home ->", wait_done())
pub.publish(String(data="inspect")); print("sent inspect")
spin(0.8)
pub.publish(String(data="stop")); print("sent stop (0.8 s after inspect)")
last = None; t = time.time()
while time.time() - t < 6:
    rclpy.spin_once(n, timeout_sec=0.05)
    if st["s"] != last: last = st["s"]; print("state:", last)
