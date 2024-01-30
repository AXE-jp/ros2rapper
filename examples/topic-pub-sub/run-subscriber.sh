#!/bin/bash

docker run -it --rm --name rossub \
  --net=host \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps-profile.xml \
  -v ./fastrtps-profile.xml:/tmp/fastrtps-profile.xml \
  ros:humble-ros-base \
  ros2 topic echo /chatter std_msgs/String
