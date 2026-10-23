import sys, time, numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
rate = float(sys.argv[1]); secs = float(sys.argv[2])
rng = np.random.default_rng(0)
base = cv2.GaussianBlur(rng.integers(0, 255, (480, 640, 3), dtype=np.uint8), (0, 0), 3)
rclpy.init(); n = Node("probe_image_pub")
pub = n.create_publisher(CompressedImage, "/probe_image", 1)
t0 = time.time(); k = 0; size = 0
while time.time() - t0 < secs:
    img = base.copy(); cv2.putText(img, f"frame {k}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    m = CompressedImage(); m.format = "jpeg"; m.data = buf.tobytes()
    m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = str(rate)
    pub.publish(m); size = len(m.data); k += 1
    time.sleep(max(0.0, t0 + k / rate - time.time()))
print(f"published {k} frames in {secs:.0f}s, target {rate} Hz, JPEG {size / 1024:.0f} KB")
