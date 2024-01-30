#!/bin/bash

# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select pub_numstr
source install/setup.bash
ros2 run pub_numstr pub
