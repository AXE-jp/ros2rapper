// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ros2_receiver.hpp"
#include "discovery_protocol_in.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "rtps.hpp"
#include "rtps_data_in.hpp"
#include "rtps_in.hpp"
#include "sedp_heartbeat_in.hpp"
#include <cstdint>

/* Cyber func=process, bdltran_option=-s, process_valid=NO */
void ros2_receiver(
    hls_stream<hls_uint<9>>  &in /* Cyber port_mode=cw_fifo */,
    hls_stream<hls_uint<10>> &out /* Cyber port_mode=axi_stream:reg_both */,
    hls_stream<rtps_data_t>
        &rtps_data_stream /* Cyber port_mode=axi_stream:reg_both */,
    hls_uint<PUB_TOPICS_MAX> pub_enable /* Cyber port_mode=in */,
    hls_uint<SUB_TOPICS_MAX> sub_enable /* Cyber port_mode=in */,
    const receiver_config_t *conf /* Cyber port_mode=in, stable_input */) {
#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out
#pragma HLS interface mode = axis port = rtps_data_stream
#pragma HLS interface mode = ap_none port = pub_enable
#pragma HLS interface mode = ap_none port = sub_enable
#pragma HLS disaggregate             variable = conf
#pragma HLS array_reshape variable = conf->ip_addr type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->ip_addr
#pragma HLS array_reshape variable = conf->subnet_mask type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->subnet_mask
#pragma HLS interface mode = ap_none port = conf->port_num_seed
#pragma HLS array_reshape variable = conf->guid_prefix type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->guid_prefix
#pragma HLS array_reshape variable = conf->pub_topic_name type = complete dim  \
    = 2
#pragma HLS array_partition variable = conf->pub_topic_name type               \
    = complete                                              dim = 1
#pragma HLS interface mode = ap_none port = conf->pub_topic_name
#pragma HLS array_partition variable = conf->pub_topic_name_len type           \
    = complete                                                  dim = 1
#pragma HLS interface mode = ap_none port = conf->pub_topic_name_len
#pragma HLS array_reshape variable = conf->pub_topic_type_name type            \
    = complete                                                 dim = 2
#pragma HLS array_partition variable = conf->pub_topic_type_name type          \
    = complete                                                   dim = 1
#pragma HLS interface mode = ap_none port = conf->pub_topic_type_name
#pragma HLS array_partition variable = conf->pub_topic_type_name_len type      \
    = complete                                                       dim = 1
#pragma HLS interface mode = ap_none port = conf->pub_topic_type_name_len
#pragma HLS array_reshape variable = conf->sub_topic_name type = complete dim  \
    = 2
#pragma HLS array_partition variable = conf->sub_topic_name type               \
    = complete                                              dim = 1
#pragma HLS interface mode = ap_none port = conf->sub_topic_name
#pragma HLS array_partition variable = conf->sub_topic_name_len type           \
    = complete                                                  dim = 1
#pragma HLS interface mode = ap_none port = conf->sub_topic_name_len
#pragma HLS array_reshape variable = conf->sub_topic_type_name type            \
    = complete                                                 dim = 2
#pragma HLS array_partition variable = conf->sub_topic_type_name type          \
    = complete                                                   dim = 1
#pragma HLS interface mode = ap_none port = conf->sub_topic_type_name
#pragma HLS array_partition variable = conf->sub_topic_type_name_len type      \
    = complete                                                       dim = 1
#pragma HLS interface mode = ap_none port = conf->sub_topic_type_name_len

    hls_stream<hls_uint<9>> stream;
    bool                    enable = (pub_enable != 0) || (sub_enable != 0);
    uint8_t                 guid_prefix[GUID_PREFIX_SIZE];
#pragma HLS array_partition variable = guid_prefix type = complete dim = 1
    uint8_t sbm_id;
    uint8_t sbm_flags;

    rtps_in(in, stream, enable, conf->guid_prefix, guid_prefix, &sbm_id,
            &sbm_flags);
    if (!stream.empty()) {
        hls_uint<9> x;
        stream.read_nb(x);
        if (sbm_id == SBM_ID_HEARTBEAT) {
            sedp_heartbeat_in(x, rtps_data_stream, guid_prefix, sbm_flags);
        } else if (sbm_id == SBM_ID_DATA) {
            hls_uint<10> y;
            rtps_data_in(x, &y, rtps_data_stream, sbm_flags, guid_prefix);
            discovery_protocol_in(
                y, rtps_data_stream, sbm_flags, pub_enable, sub_enable,
                conf->ip_addr, conf->subnet_mask, conf->port_num_seed,
                conf->pub_topic_name, conf->pub_topic_name_len,
                conf->pub_topic_type_name, conf->pub_topic_type_name_len,
                conf->sub_topic_name, conf->sub_topic_name_len,
                conf->sub_topic_type_name, conf->sub_topic_type_name_len,
                guid_prefix);
            out.write(y);
        }
    }
}
