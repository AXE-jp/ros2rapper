#!/bin/bash

# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

docker run -it --rm \
  --net=host \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps-profile.xml \
  -v ./fastrtps-profile.xml:/tmp/fastrtps-profile.xml \
  -v ./ros2_ws:/root/ros2_ws \
  -w /root/ros2_ws \
  ros:humble-ros-base \
  ./run_sub.sh
