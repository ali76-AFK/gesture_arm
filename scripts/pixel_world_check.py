#!/usr/bin/env python3
"""pixel_world_check: converts every depth pixel to base_link coordinates and checks the result against the known world.
Read-only. Run with the arm at home and the objects reset (bin empty)."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

CAM = (0.6, 0.0, 1.25)      # camera position in base_link, looking straight down (as in the detector)
rclpy.init()
n = Node("pixel_world_check")
got = {}
n.create_subscription(Image, "/camera/depth_image",
                      lambda m: got.update(d=np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()), 1)
n.create_subscription(CameraInfo, "/camera/camera_info", lambda m: got.update(K=list(m.k)), 1)
t = time.time()
while len(got) < 2 and time.time() - t < 20:
    rclpy.spin_once(n, timeout_sec=0.3)
if len(got) < 2:
    print("missing topics:", [k for k in ("d", "K") if k not in got])
    raise SystemExit(1)
d, K = got["d"], got["K"]
fx, fy, cx, cy = K[0], K[4], K[2], K[5]
v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
ok = np.isfinite(d)
X = CAM[0] - (v - cy) * d / fy
Y = CAM[1] - (u - cx) * d / fx
Z = CAM[2] - d


def rng(m):
    return f"x {X[m].min():.3f}..{X[m].max():.3f}   y {Y[m].min():.3f}..{Y[m].max():.3f}"


print(f"intrinsics fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} | valid depth pixels {ok.sum()}/{ok.size}")
print(f"expected view of a surface at z=0.20: x {CAM[0] - cy * (CAM[2] - 0.20) / fy:.3f}..{CAM[0] + cy * (CAM[2] - 0.20) / fy:.3f}   y {-cx * (CAM[2] - 0.20) / fx:.3f}..{cx * (CAM[2] - 0.20) / fx:.3f}")
tbl = ok & (np.abs(Z - 0.20) < 0.01)
print(f"table-top pixels (z within 1 cm of 0.200): {tbl.sum()}  -> {rng(tbl)}")
print(f"   their height: median {np.median(Z[tbl]):.4f} m | std {Z[tbl].std() * 1000:.1f} mm | min {Z[tbl].min():.4f} | max {Z[tbl].max():.4f}")
reg = ok & (X > 0.40) & (X < 0.70) & (Y > -0.52) & (Y < -0.15) & (Z > 0.255)
if reg.sum() > 50:
    tops = Z[reg & (Z < 0.285)]
    print(f"bin (pixels above z=0.255 near it): {rng(reg)}")
    print("   expected outer box x 0.470..0.630, y -0.410..-0.250 | differences (mm): "
          f"x {1000 * (X[reg].min() - 0.47):+.0f}/{1000 * (X[reg].max() - 0.63):+.0f}   y {1000 * (Y[reg].min() + 0.41):+.0f}/{1000 * (Y[reg].max() + 0.25):+.0f}"
          + (f" | wall top z median {np.median(tops):.4f} (expected 0.270)" if tops.size else ""))
else:
    print("bin not found in the expected region (is the scene reset, and is the arm at home?)")
