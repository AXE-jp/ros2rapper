#pragma once

#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "ros2_receiver.hpp"
#include <cstdint>

void discovery_protocol_in(
    hls_uint<10> x, hls_stream<rtps_data_t> &out, uint8_t sbm_flags,
    hls_uint<PUB_TOPICS_MAX> pub_enable, hls_uint<SUB_TOPICS_MAX> sub_enable,
    const uint8_t reader_ip_addr[4], const uint8_t subnet_mask[4],
    uint16_t      port_num_seed,
    const uint8_t pub_topic_name[PUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t pub_topic_name_len[PUB_TOPICS_MAX],
    const uint8_t pub_type_name[PUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t pub_type_name_len[PUB_TOPICS_MAX],
    const uint8_t sub_topic_name[SUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t sub_topic_name_len[SUB_TOPICS_MAX],
    const uint8_t sub_type_name[SUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t sub_type_name_len[SUB_TOPICS_MAX],
    const uint8_t guid_prefix[GUID_PREFIX_SIZE]);
