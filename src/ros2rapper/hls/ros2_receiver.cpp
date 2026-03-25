#include "ros2_receiver.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2.hpp"
#include "rtps.hpp"
#include "rtps_sbm_data_in.hpp"
#include "rtps_sbm_heartbeat_in.hpp"
#include <cstdint>

/* Cyber func=inline */
static bool compare_guid_prefix(uint16_t offset, uint8_t data,
                                const uint8_t guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS inline
    /* Cyber unroll_times=all */
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
        if ((offset == j) && (data != guid_prefix[j])) {
            return false;
        }
    }
    return true;
}

typedef enum {
    ROS2_RECEIVER_STATE_RTPS_HDR,
    ROS2_RECEIVER_STATE_SBM_HDR,
    ROS2_RECEIVER_STATE_SBM_INFO_DST,
    ROS2_RECEIVER_STATE_SBM_PAYLOAD,
    ROS2_RECEIVER_STATE_WAIT_END
} ros2_receiver_state_t;

/* Cyber func=process, bdltran_option=-s, process_valid=NO,
 * async_reset_port=rst_n- */
void ros2_receiver(
    hls_stream<hls_uint<9>>  &in /* Cyber port_mode=axi_stream:reg_both */,
    hls_stream<rtps_data_t>  &out /* Cyber port_mode=axi_stream:reg_both */,
    hls_uint<PUB_TOPICS_MAX>  pub_enable /* Cyber port_mode=in */,
    hls_uint<SUB_TOPICS_MAX>  sub_enable /* Cyber port_mode=in */,
    hls_uint<SUB_TOPICS_MAX> *sub_app_data_req /* Cyber port_mode=shared */,
    hls_uint<SUB_TOPICS_MAX> *sub_app_data_rel /* Cyber port_mode=shared */,
    hls_uint<SUB_TOPICS_MAX>  sub_app_data_grant /* Cyber port_mode=in */,
    uint8_t                   sub_app_data_0[MAX_APP_DATA_LEN],
    uint8_t                   sub_app_data_1[MAX_APP_DATA_LEN],
    uint8_t                   sub_app_data_2[MAX_APP_DATA_LEN],
    uint8_t                   sub_app_data_3[MAX_APP_DATA_LEN],
    hls_stream<uint64_t>
        &sub_app_data_recvinfo /* Cyber port_mode=axi_stream:reg_both */,
    const receiver_config_t &conf /* Cyber port_mode=in, stable_input */) {
#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out
#pragma HLS interface mode = ap_none port = pub_enable
#pragma HLS interface mode = ap_none port = sub_enable
#pragma HLS interface mode = ap_vld port = sub_app_data_req
#pragma HLS interface mode = ap_vld port = sub_app_data_rel
#pragma HLS interface mode = ap_none port = sub_app_data_grant
#pragma HLS interface mode = ap_memory port = sub_app_data_0 storage_type      \
    = ram_1p
#pragma HLS interface mode = ap_memory port = sub_app_data_1 storage_type      \
    = ram_1p
#pragma HLS interface mode = ap_memory port = sub_app_data_2 storage_type      \
    = ram_1p
#pragma HLS interface mode = ap_memory port = sub_app_data_3 storage_type      \
    = ram_1p
#pragma HLS interface mode = axis port = sub_app_data_recvinfo
#pragma HLS disaggregate          variable = conf
#pragma HLS array_reshape variable = conf.guid_prefix type = complete dim = 1
#pragma HLS interface mode = ap_none port = conf.guid_prefix
#pragma HLS array_reshape variable = conf.ip_addr type = complete dim = 1
#pragma HLS interface mode = ap_none port = conf.ip_addr
#pragma HLS array_reshape variable = conf.subnet_mask type = complete dim = 1
#pragma HLS interface mode = ap_none port = conf.subnet_mask
#pragma HLS interface mode = ap_none port = conf.port_num_seed
#pragma HLS array_reshape variable = conf.pub_topic_name type = complete dim = 1
#pragma HLS interface mode = ap_memory port = conf.pub_topic_name storage_type \
    = rom_1p                                                      latency = 1
#pragma HLS array_reshape variable = conf.pub_topic_name_len type              \
    = complete                                               dim = 1
#pragma HLS interface mode = ap_none port = conf.pub_topic_name_len
#pragma HLS array_reshape variable = conf.pub_topic_type_name type             \
    = complete                                                dim = 1
#pragma HLS interface mode = ap_memory port                                    \
    = conf.pub_topic_type_name storage_type = rom_1p latency = 1
#pragma HLS array_reshape variable = conf.pub_topic_type_name_len type         \
    = complete                                                    dim = 1
#pragma HLS interface mode = ap_none port = conf.pub_topic_type_name_len
#pragma HLS array_reshape variable = conf.sub_topic_name type = complete dim = 1
#pragma HLS interface mode = ap_memory port = conf.sub_topic_name storage_type \
    = rom_1p                                                      latency = 1
#pragma HLS array_reshape variable = conf.sub_topic_name_len type              \
    = complete                                               dim = 1
#pragma HLS interface mode = ap_none port = conf.sub_topic_name_len
#pragma HLS array_reshape variable = conf.sub_topic_type_name type             \
    = complete                                                dim = 1
#pragma HLS interface mode = ap_memory port                                    \
    = conf.sub_topic_type_name storage_type = rom_1p latency = 1
#pragma HLS array_reshape variable = conf.sub_topic_type_name_len type         \
    = complete                                                    dim = 1
#pragma HLS interface mode = ap_none port = conf.sub_topic_type_name_len
    static ros2_receiver_state_t state = ROS2_RECEIVER_STATE_RTPS_HDR;
    static uint16_t              offset = 0;
    static uint8_t               sbm_id;
    static uint8_t               sbm_flags;
    static uint16_t              sbm_length;
    bool                         sbm_le = sbm_flags & SBM_FLAGS_ENDIANNESS;

    static uint8_t src_guid_prefix[GUID_PREFIX_SIZE] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = src_guid_prefix type = complete dim = 1

    bool enable = ((pub_enable != 0) || (sub_enable != 0));

    hls_uint<9> x = in.read();
    uint8_t     data = x & 0xff;
    bool        end = x & 0x100;

    hls_uint<9> y = x;

    switch (state) {
    case ROS2_RECEIVER_STATE_RTPS_HDR:
        if (!rtps_compare_protocol(offset, data)) {
            state = ROS2_RECEIVER_STATE_WAIT_END;
            break;
        }
        /* Cyber unroll_times=all */
        for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
            if (offset == (j + RTPS_HDR_OFFSET_GUID_PREFIX)) {
                src_guid_prefix[j] = data;
            }
        }
        offset++;
        if (offset == RTPS_HDR_SIZE) {
            offset = 0;
            if (enable) {
                state = ROS2_RECEIVER_STATE_SBM_HDR;
            } else {
                state = ROS2_RECEIVER_STATE_WAIT_END;
            }
        }
        break;
    case ROS2_RECEIVER_STATE_SBM_HDR:
        switch (offset) {
        case SBM_HDR_OFFSET_SUBMESSAGE_ID:
            sbm_id = data;
            break;
        case SBM_HDR_OFFSET_FLAGS:
            sbm_flags = data;
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER:
            sbm_length = sbm_le ? data : (data << 8);
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER + 1:
            sbm_length |= sbm_le ? (data << 8) : data;
            break;
        }
        offset++;
        if (offset == SBM_HDR_SIZE) {
            offset = 0;
            if (sbm_length == 0) {
                state = ROS2_RECEIVER_STATE_SBM_HDR;
            } else if (sbm_id == SBM_ID_INFO_DST) {
                state = ROS2_RECEIVER_STATE_SBM_INFO_DST;
            } else {
                state = ROS2_RECEIVER_STATE_SBM_PAYLOAD;
            }
        }
        break;
    case ROS2_RECEIVER_STATE_SBM_INFO_DST:
        if (!compare_guid_prefix(offset, data, conf.guid_prefix)) {
            state = ROS2_RECEIVER_STATE_WAIT_END;
            break;
        }
        offset++;
        if (offset == sbm_length) {
            offset = 0;
            state = ROS2_RECEIVER_STATE_SBM_HDR;
        }
        break;
    case ROS2_RECEIVER_STATE_SBM_PAYLOAD:
        offset++;
        if (offset == sbm_length) {
            offset = 0;
            state = ROS2_RECEIVER_STATE_SBM_HDR;
            y |= hls_uint<9>(0x100);
        }
        switch (sbm_id) {
        case SBM_ID_HEARTBEAT:
            rtps_sbm_heartbeat_in(y, out, sbm_le, src_guid_prefix);
            break;
        case SBM_ID_DATA:
            rtps_sbm_data_in(y, out, pub_enable, sub_enable, sbm_flags,
                             src_guid_prefix, sub_app_data_req,
                             sub_app_data_rel, sub_app_data_grant,
                             sub_app_data_0, sub_app_data_1, sub_app_data_2,
                             sub_app_data_3, sub_app_data_recvinfo, conf);
            break;
        }
        break;
    case ROS2_RECEIVER_STATE_WAIT_END:
        break;
    }

    if (end) {
        state = ROS2_RECEIVER_STATE_RTPS_HDR;
        offset = 0;
    }
}
