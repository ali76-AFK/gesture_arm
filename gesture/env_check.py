import sys, time
import numpy as np, cv2
print("python", sys.version.split()[0], "| numpy", np.__version__, "| cv2", cv2.__version__)
import mediapipe as mp, sklearn, pandas, roslibpy
print("mediapipe", mp.__version__, "| sklearn", sklearn.__version__, "| pandas", pandas.__version__,
      "| roslibpy", getattr(roslibpy, "__version__", "?"))
for l in cv2.getBuildInformation().splitlines():
    if any(k in l for k in ("GUI", "QT", "GTK", "Wayland", "X11")):
        print("  cv2 build:", l.strip())
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("camera opened:", cap.isOpened())
for _ in range(10): cap.read()
t = time.time(); n = 0
while n < 30:
    ok, f = cap.read()
    if ok: n += 1
print("frame shape:", f.shape, "| camera fps:", round(30 / (time.time() - t), 1))
cap.release()
try:
    img = np.zeros((120, 400, 3), np.uint8); cv2.putText(img, "display test", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow("display test", img); t = time.time()
    while time.time() - t < 2: cv2.waitKey(50)
    cv2.destroyAllWindows(); print("preview window: OK")
except Exception as e:
    print("preview window: FAILED", repr(e))
