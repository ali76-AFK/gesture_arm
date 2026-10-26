# gesture_arm

Simulated UR5e arm with a two-finger gripper, driven by ROS 2 Jazzy, Gazebo Harmonic and MoveIt 2, all inside Docker.
The goal is to control the arm with hand gestures seen by a laptop webcam. The gesture side is not built yet (see Status).

## Tested on

One machine only: Ubuntu 24.04, Docker 29.9.1 with Compose v5, Intel Arc integrated GPU (Mesa), an X11/XWayland session and a built-in webcam at `/dev/video0`. Other setups are untested.

## Status

| Part | State |
|---|---|
| Gazebo world (table, bin, 4 objects, overhead RGB-D camera at z = 1.25 m that sees the whole table) | working |
| UR5e + two-finger gripper in Gazebo, MoveIt planning | working |
| `arm_control`: named poses, open/close, stop, collision scene | working |
| `object_detector`: colour segmentation, 3D position from depth | working in simulation (errors of 2 mm or less in the test) |
| `pick_place`: picks by detected position, drops in the bin | working, stop tested |
| Gesture recognition: per-hand classifiers on MediaPipe landmarks (`gesture/`) | works in a live printed test, one person, one session |
| Gesture-to-command interpreter (`gesture_interpreter.py`) | gestures drive open, close, home and stop on the simulated arm; stop mid-move, busy and stale-event rules tested with injected events (`scripts/gesture_chain_test.py`); a gesture-started pick (object selected by topic, then left thumbs up) and a gesture stop during a pick were each run once live |
| Dashboard (`dashboard/index.html`) | works live, one person, one session: mouse and right-hand control both selected and confirmed picks, STOP and abort worked. The cursor follows the palm centre through a One Euro filter. Objects are selected by a 1 s dwell or a pinch click (see Known limits) |
| Learned object detector (YOLO) | not started. Pretrained COCO YOLO did not detect the objects |
| rosbridge/roslibpy test (`scripts/probe_bridge.py`) | passes in both directions |

## Setup

Requires Docker with the Compose plugin. The compose files assume `/dev/dri` (GPU rendering) and an X11 display. `docker-compose.cam.yml` passes through `/dev/video0`, so change it if your camera is another device.

~~~bash
./scripts/setup_env.sh        # writes .env with your UID, GIDs and display values (not committed)
./scripts/run.sh build        # first build downloads several GB
./scripts/run.sh colcon       # builds src/gesture_arm_sim
~~~

## Run

Check `docker ps` is clean first, because a stale container can break startup. Use one terminal for each command.

~~~bash
./scripts/run.sh sim          # Gazebo + MoveIt + RViz, wait about 60 s
./scripts/run.sh exec 'ros2 run gesture_arm_sim arm_control.py --ros-args -p use_sim_time:=true'
./scripts/run.sh exec 'ros2 run gesture_arm_sim object_detector.py --ros-args -p use_sim_time:=true'
./scripts/run.sh exec 'ros2 run gesture_arm_sim pick_place.py --ros-args -p use_sim_time:=true'
~~~

Commands:

~~~bash
./scripts/arm_cmd.sh home|ready|inspect|open|close|stop
./scripts/pick.sh red_cube|green_cube|blue_cylinder|yellow_block
./scripts/run.sh exec 'bash /ws/scripts/reset_objects.sh'    # put the objects back
~~~

## Collect your own gesture images (not included in this repo)

No photos or camera captures are in this repository, and none should be committed. Gesture models depend on your hand, your camera and your lighting, so record your own.

1. Sit where you will use the system, with your whole hand inside the camera view.
2. Run the guided probe (needs your webcam and an internet connection to download the model):

~~~bash
docker compose -f docker-compose.yml -f docker-compose.cam.yml run --rm --user root sim bash -c '
apt-get update -qq && apt-get install -y -qq python3-venv >/dev/null 2>&1
python3 -m venv /tmp/v && . /tmp/v/bin/activate
pip -q install mediapipe 2>&1 | tail -n 1
python -u /ws/scripts/probe_gesture2.py
chown --reference=/ws/scripts /ws/gesture_probe_retest.png'
~~~

3. Hold each pose when prompted. Read the pose description in the prompt exactly, since a wrong pose gives misleading results. It prints which gesture labels came back and finger measurements for each pose.
4. It saves `gesture_probe_retest.png` (a contact sheet of what the camera saw). It contains your face and stays on your machine: it is listed in `.gitignore`. Check `git status` before every commit.
5. If you record more data later, keep it in `data/`, which is also ignored.

## Gesture control

Gestures are recognised from the 21 MediaPipe hand landmarks by one small logistic-regression model per hand. Left-hand models are trained on mirrored right-hand data plus your own left-hand session. The pretrained MediaPipe fist label never fired for the author, which is why the models exist.

| Hand | Gesture | Command |
|---|---|---|
| Right | pointing up | move the dashboard cursor |
| Right | pinch | click in the dashboard (select an object, Home, Open, Close, STOP) |
| Left | fist | close gripper |
| Left | three fingers | open gripper |
| Left | thumbs up | confirm and run the selected pick |
| Left | thumbs down | cancel |
| Left | victory | home pose |
| Either | open palm | stop |

A gesture fires only after the same label is held for 6 frames at confidence 0.8 or higher (about 0.3 s at 21 fps).

Record your own data (numbers only, no images are stored), then train and try it. `SESSION` names the recording and `HAND` keeps only frames labelled with that hand:

~~~bash
docker compose -f docker-compose.yml -f docker-compose.cam.yml run --rm --user root sim bash -c '
apt-get update -qq && apt-get install -y -qq python3-venv >/dev/null 2>&1
python3 -m venv /tmp/v && . /tmp/v/bin/activate
pip -q install mediapipe 2>&1 | tail -n 1
SESSION=s1 HAND=Right python -u /ws/scripts/record_landmarks.py
chown -R --reference=/ws/scripts /ws/data'
./scripts/gesture.sh build
./scripts/gesture.sh train --check     # held-out evaluation
./scripts/gesture.sh train             # writes models/gesture_models.npz (not committed)
./scripts/gesture.sh live --guide      # guided live test, prints which gestures fired
~~~

`gesture/train_gestures.py` maps sessions to hands in `DEFAULT_SIDE` (s1 and s2 right hand, s3 left hand). Pass `--side s4=Left` for new sessions.

Measured limits: one person, one camera, one room, three recording sessions. Relaxed hands sometimes fire a false fist or stop. Left-hand accuracy was tested only through mirrored right-hand data plus one left-hand session.

### Gestures to the arm

Needs the simulation, `arm_control` and rosbridge running (see Run). Dry run first: it logs what would be sent and sends nothing.

~~~bash
./scripts/run.sh exec 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
./scripts/run.sh exec 'ros2 run gesture_arm_sim gesture_interpreter.py --ros-args -p dry_run:=true'
./scripts/gesture.sh live --ros localhost:9090
~~~

Rules: stop works from either hand at any time. Other commands are ignored while the arm moves or a pick runs. Repeats within 1.5 s are ignored. Thumbs up runs a pick only if an object was selected in the last 30 s (set it with `/selected_object` until the dashboard exists).

### Known limits (measured)

- Aborting a pick leaves the object in the gripper. Opening the gripper afterwards drops it from the carry height.
- With the simulation, detector, `pick_place`, rosbridge, interpreter and the gesture node with preview all running, the gesture node processed about 12 to 14 frames per second (about 21 with fewer processes), so a 6-frame hold takes about 0.5 s.
- Pointer: in recorded landmarks the index tip moved about 450 page px between pointing and pinching, and the palm centre 8 to 43 px in steady poses. With the index-tip pointer, 20 of 30 pinch clicks in one live session were ignored. After switching to the palm centre and a One Euro filter the cursor was reported as much smoother, dwell selection (at least 20 selections of about 1 s) was reported always correct, and pinch selection mostly correct but delayed (the pinch event needs a 6-frame hold, about 0.35 s at 17 fps).
- `hand_state` reached the browser at 15.3 to 15.7 Hz with gaps of at most 105 ms (three readings).
- Reach (UR5e, tool pointing down, collision-checked IK, 3 seeds per cell, `scripts/reach_map.py`): at fingertip height 0.23 m 143 of 221 grid cells (x 0.20 to 1.00, y -0.6 to 0.6, 5 by 10 cm) were reachable, at 0.30 m 138, at 0.38 m 131. Nothing at x >= 0.90. A single IK seed missed many reachable cells.
- Depth to world conversion (`scripts/pixel_world_check.py`): table plane at z = 0.2000 m, bin outer box within 2 mm.
- Overhead camera at z = 1.25 m: all four objects were detected within 2 mm in x and y and 1 mm in height, and the whole table (x 0.2 to 1.0, y -0.5 to 0.5) is visible (`scripts/detect_once.py`, `scripts/pixel_world_check.py`).
- Simulation clock: in two sessions `/clock` had no ROS publisher although the clock bridge process was running. Nodes started with `use_sim_time:=true` then stalled: `arm_control` never applied the table, bin and gripper collision entries (every `home` failed with error_code -10) and the detector published nothing. A clean restart fixed it; the cause is unknown. `diag/clock_gate.sh` checks the clock; `diag/health_check.sh` and `diag/clock_check.sh` collect the evidence.
- The bin drop point is fixed and the drop height ignores objects already in the bin. Placing a third object was reported to leave the robot stuck and unresponsive; not yet understood.
- rosbridge listens on all network interfaces (port 9090) with its default launch file. Anyone who can reach the machine could read the camera feed and publish robot commands. Use a trusted network or a firewall.
- The dashboard webcam feed runs at about 7 to 8 Hz and the robot camera at about 5 Hz.

## Dashboard

Eight processes, one terminal each. Check `docker ps` is clean first. Once the simulation has been up for about 60 s, run `./scripts/run.sh exec 'bash /ws/diag/clock_gate.sh'`: it must print `CLOCK OK` before you start the other processes, and `arm_control` must then print `collision scene ... applied: True` within a few seconds.

~~~bash
./scripts/run.sh sim
./scripts/run.sh exec 'ros2 run gesture_arm_sim arm_control.py --ros-args -p use_sim_time:=true'
./scripts/run.sh exec 'ros2 run gesture_arm_sim object_detector.py --ros-args -p use_sim_time:=true'
./scripts/run.sh exec 'ros2 run gesture_arm_sim pick_place.py --ros-args -p use_sim_time:=true'
./scripts/run.sh exec 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
./scripts/run.sh exec 'ros2 run gesture_arm_sim gesture_interpreter.py'
(cd dashboard && python3 -m http.server 8080 --bind 127.0.0.1)
./scripts/gesture.sh live --ros localhost:9090 --no-preview
~~~

Open http://localhost:8080/index.html. Click an object card or the robot camera image to select it, then confirm with a left thumbs up or the Confirm button. Tick "hand control" to use the right hand: pointing moves the cursor and a pinch clicks. Esc turns hand control off. Confirm pick cannot be clicked by hand.

`dashboard/vendor/roslib.min.js` is roslib 1.4.1 (BSD-2-Clause, licence in `dashboard/vendor/roslib-LICENSE.txt`).
