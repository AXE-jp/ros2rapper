#!/bin/bash

rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select pub_numstr
source install/setup.bash
ros2 run pub_numstr pub
