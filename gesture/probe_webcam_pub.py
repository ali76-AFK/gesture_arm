"""Test publisher: webcam frames as sensor_msgs/CompressedImage over rosbridge (roslibpy), plus a text topic."""
import sys, time, base64, cv2, roslibpy
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
RATE = 10.0
c = roslibpy.Ros(host="localhost", port=9090); c.run(timeout=10)
img_t = roslibpy.Topic(c, "/probe_webcam", "sensor_msgs/CompressedImage"); img_t.advertise()
txt_t = roslibpy.Topic(c, "/probe_text", "std_msgs/String"); txt_t.advertise()
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("camera opened:", cap.isOpened(), "| publishing for", secs, "s", flush=True)
t0, k, size = time.time(), 0, 0
while time.time() - t0 < secs:
    ok, bgr = cap.read()
    if not ok: continue
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
    now = time.time()
    img_t.publish(roslibpy.Message({"header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)}, "frame_id": "webcam"},
                                    "format": "jpeg", "data": base64.b64encode(buf.tobytes()).decode()}))
    if k % 10 == 0: txt_t.publish(roslibpy.Message({"data": f"hello {k}"}))
    k += 1; size = len(buf)
    time.sleep(max(0.0, t0 + k / RATE - time.time()))
print(f"published {k} frames in {time.time() - t0:.0f} s ({k / (time.time() - t0):.1f} Hz), JPEG {size / 1024:.0f} KB", flush=True)
cap.release(); c.terminate()
