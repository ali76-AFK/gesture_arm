import os, time, urllib.request, collections
import numpy as np, cv2
import mediapipe as mp
from mediapipe.tasks.python import vision, BaseOptions

URL = "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task"
MODEL = "/tmp/gesture_recognizer.task"
if not os.path.exists(MODEL):
    urllib.request.urlretrieve(URL, MODEL)
rec = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path=MODEL), num_hands=2))

TESTS = [
 ("right_open_palm", "use your PHYSICAL RIGHT hand only: palm toward the camera, fingers spread"),
 ("left_open_palm", "use your PHYSICAL LEFT hand only: palm toward the camera, fingers spread"),
 ("left_thumbs_up", "PHYSICAL LEFT hand only: fist with the thumb pointing up"),
 ("left_victory", "PHYSICAL LEFT hand only: index and middle fingers up in a V"),
 ("left_pointing_up", "PHYSICAL LEFT hand only: only the index finger straight up"),
 ("both_open_palms", "BOTH hands up, palms toward the camera, fingers spread"),
]
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("camera opened:", cap.isOpened())
for _ in range(10): cap.read()
for name, hint in TESTS:
    print(f"\n=== NEXT: {name}  ({hint})", flush=True)
    t0, last = time.time(), 4
    while time.time() - t0 < 3:
        cap.read()
        left = 3 - int(time.time() - t0)
        if left != last: print(f"   ready in {left}...", flush=True); last = left
    print("   RECORDING", flush=True)
    n, nh, side, gest = 0, collections.Counter(), collections.Counter(), collections.Counter()
    t1 = time.time()
    while time.time() - t1 < 3:
        ok, bgr = cap.read()
        if not ok: continue
        res = rec.recognize(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
        n += 1
        nh[len(res.hand_landmarks)] += 1
        for i in range(len(res.hand_landmarks)):
            h = res.handedness[i][0].category_name
            g = res.gestures[i][0].category_name if i < len(res.gestures) and res.gestures[i] else "-"
            side[h] += 1; gest[(h, g)] += 1
    print(f"   frames={n} | hands per frame: {dict(sorted(nh.items()))} | recogniser side label: {dict(side)}")
    print(f"   gesture labels by side: {dict(gest.most_common(5))}")
cap.release()
try: rec.close()
except Exception: pass
