import os, time, csv, urllib.request
import numpy as np, cv2
import mediapipe as mp
from mediapipe.tasks.python import vision, BaseOptions

SESSION = os.environ.get("SESSION", "s1")
HAND = os.environ.get("HAND", "")     # "Left" or "Right": keep only frames with this hand label
OUT = f"/ws/data/landmarks_{SESSION}.csv"
os.makedirs("/ws/data", exist_ok=True)
URL = "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task"
MODEL = "/tmp/gesture_recognizer.task"
if not os.path.exists(MODEL):
    urllib.request.urlretrieve(URL, MODEL)
rec = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path=MODEL), num_hands=1))

GESTURES = [
    ("open_palm",   "palm toward the camera, fingers spread"),
    ("fist",        "closed fist, knuckles toward the camera"),
    ("thumbs_up",   "fist with the thumb pointing up"),
    ("thumbs_down", "fist with the thumb pointing down"),
    ("pointing_up", "ONLY the index finger straight up"),
    ("victory",     "index and middle fingers up in a V, ring and pinky curled"),
    ("three_fingers", "index, middle and ring fingers up, thumb across the palm, pinky curled"),
    ("pinch",       "thumb tip touching index fingertip, other three fingers curled"),
    ("other",       "relaxed hand, slowly change shape: half-open, waving, any pose NOT listed above"),
]
ROUNDS = [("A", "chest height, 50-60 cm from the camera"),
          ("B", "hand raised near your face"),
          ("C", "hand tilted a little and moved around the frame")]
W, H = 640, 480
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
print("camera opened:", cap.isOpened())
for _ in range(10): cap.read()

def norm(l):
    p = np.array([[q.x * W, q.y * H, q.z * W] for q in l])
    p -= p[0]
    return (p / np.linalg.norm(p[9][:2])).flatten()

header = ["session", "label", "round", "hand", "rec_label", "rec_score", "tip_x", "tip_y"] + \
         [f"{a}{i}" for i in range(21) for a in "xyz"]
new = not os.path.exists(OUT)
f = open(OUT, "a", newline=""); w = csv.writer(f)
if new: w.writerow(header)
summary = []
for rnd, rhint in ROUNDS:
    for label, hint in GESTURES:
        print(f"\n=== {label}  round {rnd}: {hint}  [{rhint}]", flush=True)
        t0, last = time.time(), 3
        while time.time() - t0 < 2:
            cap.read()
            left = 2 - int(time.time() - t0)
            if left != last: print(f"   ready in {left}...", flush=True); last = left
        print("   RECORDING", flush=True)
        n = kept = 0; t1 = time.time()
        while time.time() - t1 < 4:
            ok, bgr = cap.read()
            if not ok: continue
            n += 1
            res = rec.recognize(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
            if not res.hand_landmarks: continue
            if HAND and res.handedness[0][0].category_name != HAND: continue
            l = res.hand_landmarks[0]
            g = res.gestures[0][0] if res.gestures else None
            w.writerow([SESSION, label, rnd, res.handedness[0][0].category_name,
                        g.category_name if g else "", f"{g.score:.3f}" if g else "",
                        f"{l[8].x:.4f}", f"{l[8].y:.4f}"] + [f"{v:.5f}" for v in norm(l)])
            kept += 1
        f.flush()
        summary.append((label, rnd, n, kept))
        print(f"   frames={n} hand_frames={kept}", flush=True)
cap.release(); f.close()
try: rec.close()
except Exception: pass
print("\n=== SUMMARY (label, round, frames, hand frames) ===")
for s in summary: print("  %-14s %s  %3d  %3d" % s)
print("saved", OUT)
