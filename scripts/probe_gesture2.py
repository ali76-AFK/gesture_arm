import os, time, urllib.request
import numpy as np, cv2
import mediapipe as mp
from mediapipe.tasks.python import vision, BaseOptions

URL = "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task"
MODEL = "/tmp/gesture_recognizer.task"
if not os.path.exists(MODEL):
    urllib.request.urlretrieve(URL, MODEL)
rec = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path=MODEL), num_hands=1))

TESTS = [
    ("open_palm",     "palm facing the camera, fingers spread"),
    ("fist",          "closed fist, knuckles toward the camera"),
    ("thumbs_up",     "fist with the thumb pointing up"),
    ("thumbs_down",   "fist with the thumb pointing down"),
    ("pointing_up",   "index finger up, other fingers curled"),
    ("victory",       "index and middle finger up in a V"),
    ("pinch",         "thumb tip touching index tip, other fingers relaxed"),
    ("three_fingers", "index, middle and ring fingers up"),
    ("no_hand",       "take your hand completely out of the camera view"),
]
import os
HINTS = {
 "pointing_up":   "hand at chest height, wrist visible: ONLY the index finger straight up, other fingers curled, thumb tucked",
 "victory":       "palm toward the camera: index and middle fingers straight up and spread in a V, ring and pinky curled, thumb holding them down",
 "pinch":         "thumb tip touching index fingertip, the other three fingers CURLED into the palm (not an OK sign)",
 "three_fingers": "palm toward the camera: index, middle and ring fingers straight up, thumb folded across the palm, pinky curled",
}
if os.environ.get("POSES"):
    TESTS = [(n, HINTS.get(n, h)) for n, h in TESTS if n in os.environ["POSES"].split(",")]
CHAINS = [[0,1,2,3,4],[0,5,6,7,8],[0,9,10,11,12],[0,13,14,15,16],[0,17,18,19,20],[5,9,13,17]]

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("camera opened:", cap.isOpened())
for _ in range(10): cap.read()
H, W = 480, 640

def px(l): return np.array([l.x * W, l.y * H])

def features(l):
    w = px(l[0]); palm = np.linalg.norm(px(l[9]) - w)
    ext = [np.linalg.norm(px(l[t]) - w) / np.linalg.norm(px(l[p]) - w)
           for t, p in ((8, 6), (12, 10), (16, 14), (20, 18))]
    pinch = np.linalg.norm(px(l[4]) - px(l[8])) / palm
    thumb_dy = (px(l[4])[1] - w[1]) / palm
    thumb_out = np.linalg.norm(px(l[4]) - px(l[5])) / palm
    return ext + [pinch, thumb_dy, thumb_out]

def annotate(bgr, l, text):
    img = bgr.copy()
    if l is not None:
        for ch in CHAINS:
            cv2.polylines(img, [np.array([px(l[i]) for i in ch], np.int32)], False, (0, 255, 0), 2)
        for i in range(21): cv2.circle(img, tuple(int(v) for v in px(l[i])), 3, (0, 0, 255), -1)
    cv2.putText(img, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return cv2.resize(img, (320, 240))

tiles = []
for name, hint in TESTS:
    print(f"\n=== NEXT: {name}  ({hint})", flush=True)
    t0 = time.time(); last = 4
    while time.time() - t0 < 3:
        cap.read()
        left = 3 - int(time.time() - t0)
        if left != last: print(f"   ready in {left}...", flush=True); last = left
    print("   RECORDING - hold it", flush=True)
    labels, feats, n, nh, snap = {}, [], 0, 0, None
    t1 = time.time()
    while time.time() - t1 < 4:
        ok, bgr = cap.read()
        if not ok: continue
        res = rec.recognize(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
        n += 1
        l = res.hand_landmarks[0] if res.hand_landmarks else None
        if l is not None:
            nh += 1
            g = res.gestures[0][0] if res.gestures else None
            labels.setdefault(g.category_name if g else "(none)", []).append(g.score if g else 0.0)
            feats.append(features(l))
        if snap is None and time.time() - t1 > 2:
            top = max(labels, key=lambda k: len(labels[k])) if labels else "no hand"
            snap = annotate(bgr, l, f"{name} -> {top}")
    lab = ", ".join(f"{k}:{len(v)} (avg {np.mean(v):.2f})" for k, v in sorted(labels.items(), key=lambda kv: -len(kv[1])))
    print(f"   frames={n} hand_frames={nh} | labels: {lab or '-'}")
    if feats:
        f = np.median(np.array(feats), axis=0)
        print(f"   median ratios tip/pip  index={f[0]:.2f} middle={f[1]:.2f} ring={f[2]:.2f} pinky={f[3]:.2f} | "
              f"thumb-index dist={f[4]:.2f} thumb_dy={f[5]:+.2f} thumb_out={f[6]:.2f}")
    tiles.append(snap if snap is not None else np.zeros((240, 320, 3), np.uint8))

cap.release()
try: rec.close()
except Exception as e: print("close:", e)
while len(tiles) % 3: tiles.append(np.zeros((240, 320, 3), np.uint8))
sheet = np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)])
cv2.imwrite("/ws/gesture_probe_retest.png", sheet)
print("\nsaved /ws/gesture_probe_retest.png")
