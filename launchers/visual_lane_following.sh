#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launching app
dt-exec rosrun my_package visual_lane_following_node.py

# wait for app to end
dt-launchfile-join