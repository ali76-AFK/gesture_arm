#!/usr/bin/env bash
# puts the four objects back at their start positions (run while the sim is running)
set_pose() {
  gz service -s /world/pick_place/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 3000 --req "name: \"$1\" position: {x: $2 y: $3 z: $4} orientation: {w: 1}" | tr -d '\n'; echo "  <- $1"
}
set_pose red_cube      0.45  0.15 0.226
set_pose green_cube    0.65  0.18 0.226
set_pose blue_cylinder 0.55 -0.08 0.231
set_pose yellow_block  0.70 -0.12 0.241
