#ifndef ROS2_HPP
#define ROS2_HPP

#include <cstdint>
#include "hls.hpp"

void ros2(hls_stream<uint8_t> &in, hls_stream<uint8_t> &out);

#endif // !ROS2_HPP
