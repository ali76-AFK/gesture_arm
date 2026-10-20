#!/usr/bin/env bash
# usage: move_object.sh <model> <x> <y> <z>   (sim must be running)
gz service -s /world/pick_place/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: \"$1\" position: {x: $2 y: $3 z: $4} orientation: {w: 1}"
