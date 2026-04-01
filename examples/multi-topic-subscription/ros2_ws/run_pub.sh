#!/bin/bash

# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select py_pubsub
source install/setup.bash

if [ "$1" == "1" ]
then
    ros2 run py_pubsub pub_1
elif [ "$1" == "2" ]
then
    ros2 run py_pubsub pub_2
elif [ "$1" == "3" ]
then
    ros2 run py_pubsub pub_3
elif [ "$1" == "all" ]
then
    ros2 run py_pubsub pub_0 &
    { sleep 0.25s; ros2 run py_pubsub pub_1; } &
    { sleep 0.5s; ros2 run py_pubsub pub_2; } &
    { sleep 0.75s; ros2 run py_pubsub pub_3; } &
    wait
else
    ros2 run py_pubsub pub_0
fi
