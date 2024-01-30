#!/bin/bash

docker run -it --rm --name rospub \
  --net=host \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps-profile.xml \
  -v ./fastrtps-profile.xml:/tmp/fastrtps-profile.xml \
  -v ./ros2_ws:/root/ros2_ws \
  -w /root/ros2_ws \
  ros:humble-ros-base \
  ./run_pub.sh
