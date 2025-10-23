#ifndef ROS2_RECEIVER_HPP
#define ROS2_RECEIVER_HPP

#include <cstdint>

#include "hls.hpp"

typedef hls_uint<3> rtps_type_t;
#define RTPS_TYPE_SPDP             0
#define RTPS_TYPE_HEARTBEAT_PUB    1
#define RTPS_TYPE_HEARTBEAT_SUB    2
#define RTPS_TYPE_SEDP_PUB_SN_ONLY 3
#define RTPS_TYPE_SEDP_SUB_SN_ONLY 4
#define RTPS_TYPE_SEDP_PUB         5
#define RTPS_TYPE_SEDP_SUB         6
#define RTPS_TYPE_RM_ENDPOINT      7

typedef struct {
    rtps_type_t type;
    uint8_t     guid_prefix[12] /* Cyber array=EXPAND */;
    topic_id_t  topic_id;
    uint8_t     data[14] /* Cyber array=EXPAND */;
} rtps_data_t;

#endif // !ROS2_RECEIVER_HPP
