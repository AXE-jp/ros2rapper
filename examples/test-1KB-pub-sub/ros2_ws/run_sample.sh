#!/bin/bash

# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select sample_msgs
colcon build --packages-select sample_pubsub
source install/setup.bash
ros2 run sample_pubsub "$1"
