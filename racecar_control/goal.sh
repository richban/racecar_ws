#!/usr/bin/env sh

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp:
now, frame_id: "map"}, pose: {position: {x: 4.34, y: -0.59, z: 2.30}, orientation: {w: 1.0}}}'
