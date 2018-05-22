#!/bin/bash

set -e #exit on first error

export HUSKY_TOP_PLATE_ENABLED="false"
export HUSKY_IMU_XYZ="0.0 0.0 0.0"
export HUSKY_IMU_RPY="0.0 0.0 0.0"

roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch --screen
