#pragma once

#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "rtps.hpp"
#include <cstdint>

void sedp_reader(
    hls_uint<9> x, hls_stream<rtps_data_t> &out,
    hls_uint<PUB_TOPICS_MAX> pub_enable, hls_uint<SUB_TOPICS_MAX> sub_enable,
    builtin_ep_type_t ep_type, uint8_t seqnum, const uint8_t reader_ip_addr[4],
    const uint8_t subnet_mask[4], uint16_t port_num_seed,
    const uint8_t pub_topic_name[PUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t pub_topic_name_len[PUB_TOPICS_MAX],
    const uint8_t pub_type_name[PUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t pub_type_name_len[PUB_TOPICS_MAX],
    const uint8_t sub_topic_name[SUB_TOPICS_MAX][MAX_TOPIC_NAME_LEN],
    const uint8_t sub_topic_name_len[SUB_TOPICS_MAX],
    const uint8_t sub_type_name[SUB_TOPICS_MAX][MAX_TOPIC_TYPE_NAME_LEN],
    const uint8_t sub_type_name_len[SUB_TOPICS_MAX],
    const uint8_t src_guid_prefix[GUID_PREFIX_SIZE]);
