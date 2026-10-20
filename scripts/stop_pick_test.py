import time, rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init(); n = Node("stop_pick_test")
cmd = n.create_publisher(String, "/arm_command", 10)
pick = n.create_publisher(String, "/pick_place_command", 10)
log = []
n.create_subscription(String, "/pick_place_state", lambda m: log.append((time.time(), m.data)), 10)

def spin(sec):
    t = time.time()
    while time.time() - t < sec: rclpy.spin_once(n, timeout_sec=0.05)

while pick.get_subscription_count() == 0 or cmd.get_subscription_count() == 0: spin(0.2)
spin(1.0)
t0 = time.time()
pick.publish(String(data="green_cube")); print("sent pick green_cube")
spin(9.0)     # lands around the grasp/carry phase
cmd.publish(String(data="stop")); print(f"sent stop at +{time.time()-t0:.1f}s")
spin(6.0)
for t, s in log: print(f"+{t - t0:5.1f}s  {s}")
