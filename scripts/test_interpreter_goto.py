#!/usr/bin/env python3
"""Offline test of the go-to confirm logic in gesture_interpreter.py (ROS and the clock are stubbed; nothing is sent anywhere)."""
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

def new():
    sent.clear(); T[0] += 100.0
    return gi.Interpreter()
def state(n, wps, moving=False):
    n.on_goto_state(Str(json.dumps({"status": "idle", "moving": moving, "waypoints": wps})))
def wp(i, ok=True): return {"n": i, "u": 100.0 + i, "v": 200.0, "reachable": ok}
def ev(n, side, g): n.on_gesture(Str(json.dumps({"side": side, "gesture": g, "t": T[0]})))
fails = []
def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond: fails.append(name)
def topics(): return [t for t, d in sent]

n = new(); ev(n, "Left", "thumbs_up"); check("no goto state: nothing sent", sent == [])
n = new(); state(n, [wp(1), wp(2)]); ev(n, "Left", "thumbs_up"); check("first thumbs up only arms", "/goto/go" not in topics())
T[0] += 1.3; state(n, [wp(1), wp(2)]); ev(n, "Left", "thumbs_up"); check("second thumbs up 1.3 s later sends go", topics() == ["/goto/go"])
n = new(); state(n, [wp(1), wp(2)]); ev(n, "Left", "thumbs_up"); T[0] += 0.3; ev(n, "Left", "thumbs_up")
check("second thumbs up after 0.3 s does not send", "/goto/go" not in topics())
n = new(); state(n, [wp(1), wp(2)]); ev(n, "Left", "thumbs_up"); T[0] += 6.0; state(n, [wp(1), wp(2)]); ev(n, "Left", "thumbs_up")
check("second thumbs up after 6 s only re-arms", "/goto/go" not in topics())
n = new(); state(n, [wp(1), wp(2, ok=False)]); ev(n, "Left", "thumbs_up"); T[0] += 1.3; ev(n, "Left", "thumbs_up")
check("unreachable waypoint never starts", "/goto/go" not in topics())
n = new(); state(n, [wp(1)]); T[0] += 70.0; state(n, [wp(1)]); ev(n, "Left", "thumbs_up"); T[0] += 1.3; ev(n, "Left", "thumbs_up")
check("waypoints unchanged for 70 s do not start", "/goto/go" not in topics())
n = new(); state(n, [wp(1)], moving=True); ev(n, "Left", "thumbs_up"); T[0] += 1.3; ev(n, "Left", "thumbs_up")
check("go-to moving: confirm ignored", "/goto/go" not in topics())
n = new(); state(n, [wp(1)]); ev(n, "Left", "thumbs_up"); T[0] += 1.0; ev(n, "Left", "stop"); T[0] += 1.0; ev(n, "Left", "thumbs_up")
check("stop disarms (next thumbs up only re-arms)", "/goto/go" not in topics() and topics().count("/arm_command") == 1)
n = new(); state(n, [wp(1)]); n.on_selected(Str("red_cube")); ev(n, "Left", "thumbs_up")
check("fresh selection still picks, no go-to", topics() == ["/pick_place_command"])
n = new(); state(n, [wp(1)]); ev(n, "Left", "thumbs_down"); check("thumbs down clears waypoints when idle", topics() == ["/goto/clear"])
n = new(); state(n, [wp(1)], moving=True); ev(n, "Left", "thumbs_down"); check("thumbs down while moving sends stop only", topics() == ["/arm_command"])
n = new(); state(n, [wp(1)], moving=True); ev(n, "Left", "victory"); check("go-to moving: victory (home) ignored", sent == [])
print("ALL PASS" if not fails else "FAILED: %s" % fails)
sys.exit(1 if fails else 0)
