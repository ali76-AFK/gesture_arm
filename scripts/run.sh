#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || ./scripts/setup_env.sh
CAM="docker compose -f docker-compose.yml -f docker-compose.cam.yml"
RUNNING='docker ps -q --filter ancestor=gesture-arm/sim:dev | head -n 1'
case "$1" in
  build)       docker compose build sim ;;
  shell)       docker compose run --rm sim bash ;;
  colcon)      docker compose run --rm sim bash -c 'colcon build --symlink-install 2>&1 | tail -n 15' ;;
  gui-test)    docker compose run --rm sim bash -c 'glxinfo -B | grep -E "renderer|OpenGL version"; gz sim -v 3 -r shapes.sdf' ;;
  cam-test)    $CAM run --rm sim python3 scripts/cam_test.py ;;
  launch-info) docker compose run --rm sim bash -c 'd=$(ros2 pkg prefix ur_simulation_gz)/share/ur_simulation_gz/launch; grep -n -i "world\|gz_args\|gz_sim\|DeclareLaunchArgument" $d/*.py | head -n 60' ;;
  ur-sim)      docker compose run --rm sim ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e ;;
  world)       docker compose run --rm sim gz sim -r -v3 /ws/src/gesture_arm_sim/worlds/pick_place.sdf ;;
  exec)        shift; docker exec -it $(eval $RUNNING) bash -c "source /opt/ros/jazzy/setup.bash; [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash; $*" ;;
  sim)         docker compose run --rm sim ros2 launch gesture_arm_sim sim.launch.py ;;
  *) echo "usage: scripts/run.sh {build|shell|colcon|gui-test|cam-test|launch-info|ur-sim|world|exec <cmd>}" ;;
esac
