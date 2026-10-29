#!/usr/bin/env python3
"""Offline test of the hold-to-confirm logic in gesture_interpreter.py (ROS and the clock are stubbed; nothing is sent anywhere)."""
import importlib.util, json, sys, time, types

class Str:
    def __init__(self, data=""): self.data = data
sent, T = [], [1000.0]
time.time = lambda: T[0]
class Pub:
    def __init__(self, topic): self.topic = topic
    def publish(self, m): sent.append((self.topic, m.data))
class Log:
    def info(self, t): pass
    warn = error = info
class Node:
    def __init__(self, name): pass
    def declare_parameter(self, *a): pass
    def get_parameter(self, n): return types.SimpleNamespace(value=False)
    def create_publisher(self, t, topic, q): return Pub(topic)
    def create_subscription(self, *a): pass
    def get_logger(self): return Log()
for name, attrs in (("rclpy", {"init": lambda: None, "spin": lambda n: None}), ("rclpy.node", {"Node": Node}),
                    ("std_msgs", {}), ("std_msgs.msg", {"String": Str})):
    sys.modules[name] = types.ModuleType(name); sys.modules[name].__dict__.update(attrs)
spec = importlib.util.spec_from_file_location("gi", "src/gesture_arm_sim/scripts/gesture_interpreter.py")
gi = importlib.util.module_from_spec(spec); spec.loader.exec_module(gi)

WPS = [{"n": 1, "u": 101.0, "v": 200.0, "reachable": True}, {"n": 2, "u": 102.0, "v": 200.0, "reachable": True}]
def new():
    sent.clear(); T[0] += 100.0
    return gi.Interpreter()
def state(n, wps=None, moving=False):
    n.on_goto_state(Str(json.dumps({"status": "idle", "moving": moving, "waypoints": WPS if wps is None else wps})))
def ev(n, side, g): n.on_gesture(Str(json.dumps({"side": side, "gesture": g, "t": T[0]})))
def hs(n, label, conf=1.0, hand="Left"):
    n.on_hand_state(Str(json.dumps({"t": T[0], "hands": {hand: {"label": label, "conf": conf}}})))
def hold(n, secs, label="thumbs_up", conf=1.0, event_at=0.6, dip_at=None, hand="Left", wps=None, moving=False):
    """One hand_state message every 0.1 s for 'secs'; the arming event arrives at 'event_at' s (None = no event)."""
    for i in range(int(round(secs * 10))):
        if i % 10 == 0:
            state(n, wps, moving)
        hs(n, label, 0.71 if dip_at == i else conf, hand)
        if event_at is not None and i == int(round(event_at * 10)):
            ev(n, "Left", "thumbs_up")
        T[0] += 0.1
fails = []
def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond: fails.append(name)
def topics(): return [t for t, d in sent]

n = new(); hold(n, 2.5); check("armed by event, held 2.5 s: go sent once", topics() == ["/goto/go"])
n = new(); hold(n, 8.0); check("held 8 s: still exactly one go", topics() == ["/goto/go"])
n = new(); hold(n, 1.5); check("dropped after 1.5 s: no go", topics() == [])
n = new(); hold(n, 3.0, dip_at=12); check("one 0.71-confidence frame does not break the hold", topics() == ["/goto/go"])
n = new(); hold(n, 3.0, conf=0.7); check("confidence 0.7 all along: nothing", topics() == [])
n = new(); hold(n, 3.0, event_at=None); check("hold without arming event: no go", topics() == [])
n = new(); hold(n, 1.0); T[0] += 0.7; hold(n, 1.5, event_at=None)
check("0.7 s break restarts the hold: no go", topics() == [])
n = new(); hold(n, 1.0); ev(n, "Left", "stop"); hold(n, 2.0, event_at=None)
check("stop event disarms: no go", topics() == ["/arm_command"])
n = new(); hold(n, 2.5, moving=True); check("go-to moving: no go", topics() == [])
n = new(); n.on_selected(Str("red_cube")); hold(n, 2.5)
check("fresh object selection: hold does not start a go-to (pick path only)", "/goto/go" not in topics())
n = new(); hold(n, 2.5, wps=[]); check("no waypoints: no go", topics() == [])
n = new(); hold(n, 2.5, wps=[dict(WPS[0], reachable=False)]); check("unreachable waypoint: no go", topics() == [])
n = new(); hold(n, 2.5, hand="Right"); check("right hand thumbs_up label: nothing", topics() == [])
n = new(); hold(n, 2.5, label="fist"); check("left fist held: nothing", topics() == [])
n = new(); hold(n, 2.5); hold(n, 0.8, label="fist", event_at=None); T[0] += 2.0; sent.clear(); hold(n, 2.5)
check("a second separate hold after the cooldown starts again", topics() == ["/goto/go"])
n = new(); hold(n, 2.5, event_at=None); ev(n, "Left", "thumbs_up"); T[0] += 0.1; hs(n, "thumbs_up")
check("arming after the hold already ran 2.5 s does not fire at once", topics() == [])
for _ in range(12):
    state(n); hs(n, "thumbs_up"); T[0] += 0.1
check("...but fires 1 s after the arm if the hold continues", topics() == ["/goto/go"])
print("ALL PASS" if not fails else "FAILED: %s" % fails)
sys.exit(1 if fails else 0)
