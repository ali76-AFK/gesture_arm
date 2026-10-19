import time, numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class Grab(Node):
    def __init__(self):
        super().__init__("grab_frame")
        self.rgb = self.depth = None
        self.create_subscription(Image, "/camera/image", self.on_rgb, 1)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, 1)
    def on_rgb(self, m):
        self.rgb = (m.encoding, np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1).copy())
    def on_depth(self, m):
        self.depth = (m.encoding, np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy())

rclpy.init()
n = Grab(); t0 = time.time()
while (n.rgb is None or n.depth is None) and time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.5)
if n.rgb is None or n.depth is None:
    print("NO FRAMES  rgb:", n.rgb is not None, " depth:", n.depth is not None); raise SystemExit(1)
enc, rgb = n.rgb; enc_d, d = n.depth
cv2.imwrite("/ws/camera_rgb.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
dv = ((np.clip(d, 0.5, 1.0) - 0.5) / 0.5 * 255).astype(np.uint8)
cv2.imwrite("/ws/camera_depth.png", dv)
f = d[np.isfinite(d)]
print(f"rgb: {rgb.shape} enc={enc} | depth: {d.shape} enc={enc_d}")
print(f"depth min={f.min():.3f} max={f.max():.3f} center={d[d.shape[0]//2, d.shape[1]//2]:.3f}  (expect ~0.75 to 0.80 m)")
