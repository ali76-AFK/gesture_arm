#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
[ -f .env ] || ./scripts/setup_env.sh
DC="docker compose -f docker-compose.yml -f docker-compose.cam.yml"
case "$1" in
  build) $DC build gesture ;;
  check) $DC run --rm gesture python gesture/env_check.py ;;
  train) shift; $DC run --rm gesture python gesture/train_gestures.py "$@" ;;
  live)  shift; $DC run --rm gesture python gesture/gesture_node.py "$@" ;;
  shell) $DC run --rm gesture bash ;;
  *) echo "usage: scripts/gesture.sh {build|check|train [--check]|live [--guide] [--no-preview]|shell}" ;;
esac
