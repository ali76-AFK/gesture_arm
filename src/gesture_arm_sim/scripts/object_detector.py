#!/usr/bin/env python3
"""object_detector: colour segmentation on the overhead RGB-D camera -> /detections (JSON)."""
import json, time
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String

CAM = (0.6, 0.0, 1.0)     # camera position in base_link; looks straight down (matches make_world.py)
COLORS = {
    "red_cube":      [((0, 150, 70), (6, 255, 255)), ((172, 150, 70), (179, 255, 255))],
    "green_cube":    [((40, 100, 60), (85, 255, 255))],
    "blue_cylinder": [((100, 150, 60), (130, 255, 255))],
    "yellow_block":  [((20, 150, 80), (35, 255, 255))],
}


def detect(rgb, depth, K):
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    found = {}
    for name, ranges in COLORS.items():
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
        if n < 2:
            continue
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[i, cv2.CC_STAT_AREA] < 150:
            continue
        ys, xs = np.nonzero(lab == i)
        d = depth[ys, xs]
        ok = np.isfinite(d)
        ys, xs, d = ys[ok], xs[ok], d[ok]
        if len(d) < 50:
            continue
        top = d < d.min() + 0.015                      # top face = closest pixels
        dz = float(np.median(d[top]))
        u, v = float(xs[top].mean()), float(ys[top].mean())
        found[name] = dict(
            x=float(CAM[0] - (v - cy) * dz / fy), y=float(CAM[1] - (u - cx) * dz / fx),
            top_z=float(CAM[2] - dz),
            wx=float((ys[top].max() - ys[top].min() + 1) * dz / fy),
            wy=float((xs[top].max() - xs[top].min() + 1) * dz / fx),
            px=[u, v], area=int(stats[i, cv2.CC_STAT_AREA]))
    return found


class Detector(Node):
    def __init__(self):
        super().__init__("object_detector")
        self.rgb = self.depth = self.K = None
        self.create_subscription(Image, "/camera/image", self.on_rgb, 1)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, 1)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 1)
        self.pub = self.create_publisher(String, "/detections", 10)
        self.img_pub = self.create_publisher(CompressedImage, "/detections/image/compressed", 1)
        self.create_timer(0.2, self.tick)
        self.get_logger().info("object_detector running (colour segmentation) -> /detections")

    def on_rgb(self, m):
        self.rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()

    def on_depth(self, m):
        self.depth = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()

    def on_info(self, m):
        self.K = list(m.k)

    def tick(self):
        if self.rgb is None or self.depth is None or self.K is None:
            return
        found = detect(self.rgb, self.depth, self.K)
        self.pub.publish(String(data=json.dumps({"t": time.time(), "objs": found})))
        if self.img_pub.get_subscription_count():
            img = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
            for name, f in found.items():
                u, v = int(f["px"][0]), int(f["px"][1])
                cv2.circle(img, (u, v), 5, (255, 255, 255), 2)
                cv2.putText(img, name, (u + 8, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                m = CompressedImage()
                m.format = "jpeg"
                m.data = buf.tobytes()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = "camera_link"
                self.img_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Detector())


if __name__ == "__main__":
    main()
