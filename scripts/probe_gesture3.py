import os, time, urllib.request
import numpy as np, cv2
import mediapipe as mp
from mediapipe.tasks.python import vision, BaseOptions

URL = "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task"
MODEL = "/tmp/gesture_recognizer.task"
OUT = os.environ.get("OUT", "/ws/gesture_probe_check.png")
if not os.path.exists(MODEL):
    urllib.request.urlretrieve(URL, MODEL)
rec = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path=MODEL), num_hands=1))

FIST = [
 ("fist_near", "closed fist, knuckles toward the camera, close to the camera (hand looks big)"),
 ("fist_mid", "closed fist, knuckles toward the camera, chest height, 50-60 cm away"),
 ("fist_side", "closed fist, thumb wrapped over the fingers, back of hand toward the camera, chest height"),
]
TESTS = [
 ("pinch", "thumb tip touching index fingertip, other three fingers CURLED into the palm, chest height"),
 ("ok_sign", "thumb tip touching index fingertip, middle, ring and pinky STRAIGHT UP (OK sign), chest height"),
 ("fist", "closed fist, knuckles toward the camera, chest height"),
 ("pointing_up", "ONLY the index finger straight up, other fingers curled, chest height"),
 ("victory", "palm toward the camera, chest height, 50-60 cm away: index and middle fingers up in a V, ring and pinky curled"),
 ("victory_close", "the same V sign, but raise the hand near your face so it looks bigger in the picture"),
 ("three_fingers", "chest height, 50-60 cm away: index, middle and ring fingers up, thumb across the palm, pinky curled"),
 ("three_fingers_close", "the same three fingers, raised near your face so the hand looks bigger"),
 ("open_palm", "palm facing the camera, fingers spread, chest height"),
]
if os.environ.get("FIST"):
    TESTS = FIST
CHAINS = [[0,1,2,3,4],[0,5,6,7,8],[0,9,10,11,12],[0,13,14,15,16],[0,17,18,19,20],[5,9,13,17]]
H, W = 480, 640

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
print("camera opened:", cap.isOpened())
for _ in range(10): cap.read()

def px(p): return np.array([p.x * W, p.y * H])

def feats(l):
    w = px(l[0]); palm = np.linalg.norm(px(l[9]) - w)
    ext = [np.linalg.norm(px(l[t]) - w) / np.linalg.norm(px(l[p]) - w) for t, p in ((8, 6), (12, 10), (16, 14), (20, 18))]
    ti = np.linalg.norm(px(l[4]) - px(l[8])) / palm
    edge = any(p.x < 0.02 or p.x > 0.98 or p.y < 0.02 or p.y > 0.98 for p in l)
    return ext, ti, palm / H, edge

def annotate(bgr, l, text):
    img = bgr.copy()
    if l is not None:
        for ch in CHAINS:
            cv2.polylines(img, [np.array([px(l[i]) for i in ch], np.int32)], False, (0, 255, 0), 2)
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
    labels, E, TI, SZ, ED, FIRE, CNT = {}, [], [], [], [], [], []
    n, snap, t1 = 0, None, time.time()
    while time.time() - t1 < 4:
        ok, bgr = cap.read()
        if not ok: continue
        res = rec.recognize(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
        n += 1
        l = res.hand_landmarks[0] if res.hand_landmarks else None
        if l is not None:
            g = res.gestures[0][0] if res.gestures else None
            labels.setdefault(g.category_name if g else "(none)", []).append(g.score if g else 0.0)
            ext, ti, size, edge = feats(l)
            E.append(ext); TI.append(ti); SZ.append(size); ED.append(edge)
            CNT.append(sum(e > 1.15 for e in ext))
            FIRE.append(ti < 0.25 and ext[0] > 0.85 and ext[1] < 0.85)
        if snap is None and time.time() - t1 > 2:
            top = max(labels, key=lambda k: len(labels[k])) if labels else "no hand"
            snap = annotate(bgr, l, f"{name} -> {top}")
    lab = ", ".join(f"{k}:{len(v)} (avg {np.mean(v):.2f}, min {np.min(v):.2f})" for k, v in sorted(labels.items(), key=lambda kv: -len(kv[1])))
    print(f"   frames={n} hand_frames={len(E)} | labels: {lab or '-'}")
    if E:
        hist = {}
        for c in CNT: hist[c] = hist.get(c, 0) + 1
        e = np.median(np.array(E), axis=0)
        print(f"   palm size (fraction of frame height) median={np.median(SZ):.2f} | touching frame edge in {100 * np.mean(ED):.0f}% of hand frames")
        print(f"   extended fingers (ratio>1.15) per frame: {dict(sorted(hist.items()))} | median ratios index={e[0]:.2f} middle={e[1]:.2f} ring={e[2]:.2f} pinky={e[3]:.2f} | thumb-index={np.median(TI):.2f}")
        print(f"   candidate pinch rule fires in {100 * np.mean(FIRE):.0f}% of hand frames")
    tiles.append(snap if snap is not None else np.zeros((240, 320, 3), np.uint8))

cap.release()
try: rec.close()
except Exception as ex: print("close:", ex)
while len(tiles) % 3: tiles.append(np.zeros((240, 320, 3), np.uint8))
cv2.imwrite(OUT, np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]))
print("\nsaved", OUT)
