import time, numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

CAM = (0.6, 0.0, 1.0)     # camera position in base_link; looks straight down (matches make_world.py)
# HSV ranges (OpenCV hue 0-179). Table is tan: hue ~15, saturation lower than the objects.
COLORS = {
    "red_cube":      [((0, 150, 70), (6, 255, 255)), ((172, 150, 70), (179, 255, 255))],
    "green_cube":    [((40, 100, 60), (85, 255, 255))],
    "blue_cylinder": [((100, 150, 60), (130, 255, 255))],
    "yellow_block":  [((20, 150, 80), (35, 255, 255))],
}
# ground truth from make_world.py: (x, y, top z)
TRUTH = {"red_cube": (0.45, 0.15, 0.251), "green_cube": (0.65, 0.18, 0.251),
         "blue_cylinder": (0.55, -0.08, 0.261), "yellow_block": (0.70, -0.12, 0.281)}

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
        d = depth[ys, xs]; ok = np.isfinite(d)
        ys, xs, d = ys[ok], xs[ok], d[ok]
        if len(d) < 50:
            continue
        top = d < d.min() + 0.015                      # top face = closest pixels
        dz = float(np.median(d[top])); u, v = xs[top].mean(), ys[top].mean()
        found[name] = dict(
            x=CAM[0] - (v - cy) * dz / fy, y=CAM[1] - (u - cx) * dz / fx, top_z=CAM[2] - dz,
            wx=(ys[top].max() - ys[top].min() + 1) * dz / fy,
            wy=(xs[top].max() - xs[top].min() + 1) * dz / fx, px=(u, v), area=int(stats[i, cv2.CC_STAT_AREA]))
    return found

if __name__ == "__main__":
    rclpy.init(); n = Node("detect_once"); got = {}
    n.create_subscription(Image, "/camera/image", lambda m: got.update(rgb=np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()), 1)
    n.create_subscription(Image, "/camera/depth_image", lambda m: got.update(depth=np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()), 1)
    n.create_subscription(CameraInfo, "/camera/camera_info", lambda m: got.update(K=list(m.k)), 1)
    t = time.time()
    while len(got) < 3 and time.time() - t < 20: rclpy.spin_once(n, timeout_sec=0.3)
    if len(got) < 3: print("missing topics, got:", list(got)); raise SystemExit(1)
    found = detect(got["rgb"], got["depth"], got["K"])
    print(f"intrinsics fx={got['K'][0]:.1f} fy={got['K'][4]:.1f} cx={got['K'][2]:.1f} cy={got['K'][5]:.1f}")
    print(f"{'object':14} {'x':>6} {'y':>6} {'topz':>6} {'wx cm':>6} {'wy cm':>6} | err x,y mm   topz mm")
    img = cv2.cvtColor(got["rgb"], cv2.COLOR_RGB2BGR)
    for name, (tx, ty, tz) in TRUTH.items():
        f = found.get(name)
        if f is None:
            print(f"{name:14} NOT FOUND"); continue
        print(f"{name:14} {f['x']:6.3f} {f['y']:6.3f} {f['top_z']:6.3f} {f['wx']*100:6.1f} {f['wy']*100:6.1f} |"
              f" {(f['x']-tx)*1000:5.0f},{(f['y']-ty)*1000:5.0f}   {(f['top_z']-tz)*1000:5.0f}")
        cv2.circle(img, (int(f["px"][0]), int(f["px"][1])), 5, (255, 255, 255), 2)
        cv2.putText(img, name, (int(f["px"][0]) + 8, int(f["px"][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imwrite("/ws/detect_debug.png", img)
