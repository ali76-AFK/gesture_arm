import sys, time, collections, roslibpy
secs = float(sys.argv[1])
c = roslibpy.Ros(host="localhost", port=9090); c.run(timeout=10)
rows = collections.defaultdict(list)
def cb(m):
    now = time.time(); h = m["header"]
    rows[h["frame_id"]].append((now, now - (h["stamp"]["sec"] + h["stamp"]["nanosec"] * 1e-9), len(m["data"])))
t = roslibpy.Topic(c, "/probe_image", "sensor_msgs/CompressedImage"); t.subscribe(cb)
print("subscribed, waiting for the publisher...", flush=True)
time.sleep(secs); t.unsubscribe(); c.terminate()
if not rows: print("NO MESSAGES RECEIVED")
for k in sorted(rows, key=float):
    r = rows[k]; span = r[-1][0] - r[0][0]
    print(f"target {k:>5} Hz: {len(r)} frames, {len(r) / span if span else 0:.1f} Hz measured, "
          f"latency mean {1000 * sum(x[1] for x in r) / len(r):.0f} ms max {1000 * max(x[1] for x in r):.0f} ms, "
          f"{sum(x[2] for x in r) / len(r) / 1024:.0f} KB base64 per frame")
