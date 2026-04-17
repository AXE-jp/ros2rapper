#!/bin/bash

# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select py_pubsub
source install/setup.bash
ros2 run py_pubsub sub_0 &
ros2 run py_pubsub sub_1 &
ros2 run py_pubsub sub_2 &
ros2 run py_pubsub sub_3 &
wait
