# gesture_arm

Simulated UR5e arm with a two-finger gripper, driven by ROS 2 Jazzy, Gazebo Harmonic and MoveIt 2, all inside Docker.
The goal is to control the arm with hand gestures seen by a laptop webcam. The gesture side is not built yet (see Status).

## Tested on

One machine only: Ubuntu 24.04, Docker 29.9.1 with Compose v5, Intel Arc integrated GPU (Mesa), an X11/XWayland session and a built-in webcam at `/dev/video0`. Other setups are untested.

## Status

| Part | State |
|---|---|
| Gazebo world (table, bin, 4 objects, overhead RGB-D camera) | working |
| UR5e + two-finger gripper in Gazebo, MoveIt planning | working |
| `arm_control`: named poses, open/close, stop, collision scene | working |
| `object_detector`: colour segmentation, 3D position from depth | working in simulation (errors of 2 mm or less in the test) |
| `pick_place`: picks by detected position, drops in the bin | working, stop tested |
| Gesture recognition (MediaPipe) | probed only, not built |
| Gesture-to-command interpreter, dashboard | not started |
| Learned object detector (YOLO) | not started. Pretrained COCO YOLO did not detect the objects |
| rosbridge/roslibpy test (`scripts/probe_bridge.py`) | failed, cause unknown |

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
chown --reference=/ws/scripts /ws/gesture_probe.png'
~~~

3. Hold each pose when prompted. Read the pose description in the prompt exactly, since a wrong pose gives misleading results. It prints which gesture labels came back and finger measurements for each pose.
4. It saves `gesture_probe.png` (a contact sheet of what the camera saw). It contains your face and stays on your machine: it is listed in `.gitignore`. Check `git status` before every commit.
5. If you record more data later, keep it in `data/`, which is also ignored.
