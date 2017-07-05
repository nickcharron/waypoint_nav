#!/bin/bash

set -e #exit on first error

export HUSKY_TOP_PLATE_ENABLED="false"

roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch
