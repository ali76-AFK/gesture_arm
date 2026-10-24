#!/usr/bin/env python3
"""gesture_node: webcam -> MediaPipe landmarks -> per-hand classifier -> hold filter -> gesture events (printed)."""
import argparse, base64, json, sys, time, urllib.request
from pathlib import Path
import numpy as np, cv2
import mediapipe as mp
from mediapipe.tasks.python import vision, BaseOptions
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import W, H, Softmax, norm_landmarks

ROOT = Path(__file__).resolve().parent.parent
URL = "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task"
CHAINS = [[0,1,2,3,4],[0,5,6,7,8],[0,9,10,11,12],[0,13,14,15,16],[0,17,18,19,20],[5,9,13,17]]
STEP = 7.0
GUIDE = [
 ("LEFT hand: closed fist", "Left", "fist"),
 ("LEFT hand: index, middle, ring up, thumb across palm, pinky curled", "Left", "three_fingers"),
 ("LEFT hand: thumbs up", "Left", "thumbs_up"),
 ("LEFT hand: thumbs down", "Left", "thumbs_down"),
 ("LEFT hand: victory V", "Left", "victory"),
 ("RIGHT hand: ONLY the index finger up", "Right", "pointing_up"),
 ("RIGHT hand: pinch, thumb tip on index tip, other fingers curled", "Right", "pinch"),
 ("RIGHT hand: open palm", "Right", "stop"),
 ("LEFT hand: open palm", "Left", "stop"),
 ("BOTH hands relaxed in view, move slowly, NO gesture", None, None),
 ("take BOTH hands OUT of view", None, None),
]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", default=str(ROOT / "models" / "gesture_models.npz"))
    ap.add_argument("--hold", type=int, default=6)
    ap.add_argument("--thr", type=float, default=0.8)
    ap.add_argument("--palm-score", type=float, default=0.5)
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--guide", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--ros", default="", help="rosbridge host:port, e.g. localhost:9090")
    a = ap.parse_args()
    import json
    pub = img_pub = hs_pub = None
    if a.ros:
        import roslibpy
        host, port = a.ros.split(":")
        client = roslibpy.Ros(host=host, port=int(port))
        try:
            client.run(timeout=10)
            pub = roslibpy.Topic(client, "/gesture", "std_msgs/String"); pub.advertise()
            print("rosbridge connected, publishing /gesture", flush=True)
            img_pub = roslibpy.Topic(client, "/dashboard/webcam/compressed", "sensor_msgs/CompressedImage"); img_pub.advertise()
            hs_pub = roslibpy.Topic(client, "/hand_state", "std_msgs/String"); hs_pub.advertise()
        except Exception as ex:
            print("rosbridge connection FAILED, continuing without ROS:", ex, flush=True)
    err = {"n": 0}
    def send(topic, payload):
        if topic is None:
            return
        try:
            topic.publish(roslibpy.Message(payload))
        except Exception as ex:
            err["n"] += 1
            if err["n"] == 1:
                print("publish failed:", ex, flush=True)
    z = np.load(a.models, allow_pickle=False)
    M = {"Left": Softmax(z["left_coef"], z["left_intercept"], z["left_classes"]),
         "Right": Softmax(z["right_coef"], z["right_intercept"], z["right_classes"])}
    task = ROOT / "models" / "gesture_recognizer.task"
    if not task.exists():
        task.parent.mkdir(exist_ok=True); urllib.request.urlretrieve(URL, str(task))
    rec = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
        base_options=BaseOptions(model_asset_path=str(task)), num_hands=2))
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    print("camera opened:", cap.isOpened(), "| hold", a.hold, "frames | model confidence >=", a.thr, "| palm score >=", a.palm_score)
    for _ in range(10): cap.read()
    duration = a.duration or (len(GUIDE) * STEP if a.guide else 0)
    preview = not a.no_preview
    run = {"Left": (None, 0), "Right": (None, 0)}
    events, ms, nframes, last_step, last_status = [], [], 0, -1, 0.0
    last_event, pointer = "", None
    last_img = 0.0
    t0 = time.time()
    while True:
        el = time.time() - t0
        if duration and el >= duration: break
        ok, bgr = cap.read()
        if not ok: continue
        if bgr.shape[:2] != (H, W): bgr = cv2.resize(bgr, (W, H))
        step = min(int(el // STEP), len(GUIDE) - 1) if a.guide else -1
        if a.guide and step != last_step:
            last_step = step; print(f"\n>>> STEP {step + 1}/{len(GUIDE)}  +{el:4.0f}s: {GUIDE[step][0]}", flush=True)
        tp = time.time()
        res = rec.recognize(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
        best = {}
        for i in range(len(res.hand_landmarks)):
            side, sc = res.handedness[i][0].category_name, res.handedness[i][0].score
            if side not in best or sc > best[side][0]: best[side] = (sc, i)
        cand, shown, pointer = {}, {}, None
        for side, (_, i) in best.items():
            lm = res.hand_landmarks[i]
            p = M[side].proba(norm_landmarks(lm)); k = int(p.argmax()); label, conf = str(M[side].classes[k]), float(p[k])
            pre = res.gestures[i][0] if i < len(res.gestures) and res.gestures[i] else None
            pre_palm = pre is not None and pre.category_name == "Open_Palm" and pre.score >= a.palm_score
            if (label == "open_palm" and conf >= a.thr) or pre_palm:
                cand[side] = ("stop", conf if label == "open_palm" else pre.score)
            elif conf >= a.thr and label not in ("other", "open_palm"):
                cand[side] = (label, conf)
            shown[side] = (label, conf, pre.category_name if pre else "-", lm)
            if side == "Right" and label == "pointing_up" and conf >= a.thr:
                pointer = (lm[8].x, lm[8].y)
        if hs_pub is not None and shown:
            send(hs_pub, {"data": json.dumps({"t": time.time(), "hands": {
                sd: {"label": v[0], "conf": round(float(v[1]), 3), "x": round(float(v[3][8].x), 4), "y": round(float(v[3][8].y), 4), "px": round(float(sum(v[3][i].x for i in (0, 5, 9, 13, 17)) / 5), 4), "py": round(float(sum(v[3][i].y for i in (0, 5, 9, 13, 17)) / 5), 4)}
                for sd, v in shown.items()}})})
        if img_pub is not None and time.time() - last_img >= 0.1:
            last_img = time.time()
            dimg = bgr.copy()
            for sd, v in shown.items():
                for ch in CHAINS:
                    cv2.polylines(dimg, [np.array([[v[3][j].x * W, v[3][j].y * H] for j in ch], np.int32)], False, (0, 255, 0), 2)
            okj, buf = cv2.imencode(".jpg", dimg, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if okj:
                tnow = time.time()
                send(img_pub, {"header": {"stamp": {"sec": int(tnow), "nanosec": int((tnow % 1) * 1e9)}, "frame_id": "webcam"},
                               "format": "jpeg", "data": base64.b64encode(buf.tobytes()).decode()})
        for side in ("Left", "Right"):
            c = cand.get(side)
            if c is None:
                run[side] = (None, 0); continue
            prev, n = run[side]
            n = n + 1 if c[0] == prev else 1
            run[side] = (c[0], n)
            if n == a.hold:
                events.append((el, side, c[0], c[1], step))
                last_event = f"{side} {c[0]}"
                print(f"  EVENT +{el:5.1f}s  {side:5s} {c[0]:14s} conf={c[1]:.2f}", flush=True)
                if pub is not None:
                    try:
                        pub.publish(roslibpy.Message({"data": json.dumps({"t": time.time(), "side": side, "gesture": c[0], "conf": round(float(c[1]), 3)})}))
                    except Exception as ex:
                        print("publish failed:", ex, flush=True)
        ms.append((time.time() - tp) * 1000); nframes += 1
        if el - last_status >= 3 and not a.guide:
            last_status = el
            print(f"  [{el:5.0f}s] fps={nframes / el:4.1f} seen=" + (", ".join(f"{s}:{v[0]}({v[1]:.2f})" for s, v in shown.items()) or "none"), flush=True)
        if preview:
            try:
                img = bgr.copy()
                for side, (label, conf, pre, lm) in shown.items():
                    for ch in CHAINS:
                        cv2.polylines(img, [np.array([[lm[j].x * W, lm[j].y * H] for j in ch], np.int32)], False, (0, 255, 0), 2)
                y = 24
                for side in ("Left", "Right"):
                    if side in shown:
                        txt = f"{side}: {shown[side][0]} {shown[side][1]:.2f} (pretrained {shown[side][2]}) hold {run[side][1]}/{a.hold}"
                        cv2.putText(img, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1); y += 22
                if last_event: cv2.putText(img, "last event: " + last_event, (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                if a.guide: cv2.putText(img, GUIDE[step][0][:60], (8, H - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
                cv2.imshow("gesture (unmirrored camera view, q to quit)", img)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27): break
            except cv2.error as e:
                print("preview disabled:", e); preview = False
    cap.release(); cv2.destroyAllWindows()
    if pub is not None:
        client.terminate()
    try: rec.close()
    except Exception: pass
    el = time.time() - t0
    print(f"\nframes={nframes} | {nframes / el:.1f} fps | processing {np.mean(ms):.1f} ms/frame (max {np.max(ms):.0f})")
    print("events:", len(events))
    if a.guide:
        print("\n=== GUIDED RESULT ===")
        for k, (txt, side, exp) in enumerate(GUIDE):
            ev = [e for e in events if e[4] == k]
            got = ", ".join(f"{e[1]}:{e[2]}@{e[0]:.1f}s" for e in ev) or "-"
            if exp is None:
                verdict = "OK (nothing fired)" if not ev else f"FALSE FIRES: {len(ev)}"
            else:
                hit = [e for e in ev if e[2] == exp and (exp == "stop" or e[1] == side)]
                verdict = ("OK" if hit else "MISSED") + (f", extra {len(ev) - len(hit)}" if len(ev) > len(hit) else "")
            print(f"  {k + 1:2d}. {txt[:46]:46s} -> {verdict:20s} fired: {got}")

if __name__ == "__main__":
    main()
