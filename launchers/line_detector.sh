#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launching app
dt-exec rosrun my_package line_detector_node.py

# wait for app to end
dt-launchfile-join
