#include "common.hpp"
#include "duration.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include "message_metadata.hpp"
#include "ros2.hpp"
#include "slip.hpp"
#include "spdp.hpp"
#include "udp.hpp"
#include <cstdint>

#define USE_FIFOIF_ETHERNET

/* Cyber func=inline */
static uint16_t spdp_writer_out(const sender_config_t    *conf,
                                const message_metadata_t *msg_metadata,
                                uint8_t                   tx_buf[TX_BUF_LEN]) {
    uint8_t metatraffic_port[2];
#pragma HLS array_partition variable = metatraffic_port type = complete dim = 1
    uint8_t default_port[2];
#pragma HLS array_partition variable = default_port type = complete dim = 1
    duration lease_duration;

    deserialize_spdp_metadata(metatraffic_port, default_port, &lease_duration,
                              msg_metadata);

    ip_set_header(conf->ip_addr, msg_metadata->dst_addr, IP_HDR_TTL_MULTICAST,
                  SPDP_WRITER_UDP_PKT_LEN, tx_buf);

    udp_set_header(conf->node_udp_port, msg_metadata->dst_port,
                   SPDP_WRITER_RTPS_PKT_LEN, tx_buf + IP_HDR_SIZE);

    spdp_writer(conf->guid_prefix, conf->ip_addr, metatraffic_port,
                conf->ip_addr, default_port, lease_duration,
                tx_buf + (IP_HDR_SIZE + UDP_HDR_SIZE), conf->node_name,
                conf->node_name_len, msg_metadata->now);

    return SPDP_WRITER_IP_PKT_LEN;
}

/* Cyber func=process, bdltran_option=-s, process_valid=NO */
void ros2_sender(
    hls_stream<message_metadata_t> &in /* Cyber port_mode=cw_fifo */,
    hls_stream<uint8_t>            &out /* Cyber port_mode=cw_fifo */,
    const sender_config_t *conf /* Cyber port_mode=in, stable_input */) {
#pragma HLS interface mode = ap_fifo port = in
#pragma HLS interface mode = ap_fifo port = out
#pragma HLS disaggregate             variable = conf
#pragma HLS array_reshape variable = conf->ip_addr type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->ip_addr
#pragma HLS array_reshape variable = conf->node_name type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->node_name
#pragma HLS interface mode = ap_none port = conf->node_name_len
#pragma HLS array_reshape variable = conf->node_udp_port type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->node_udp_port
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
#pragma HLS interface mode = ap_ctrl_none port = return

    message_metadata_t msg_metadata;
#pragma HLS array_partition variable = msg_metadata.dst_addr type              \
    = complete                                               dim = 1
#pragma HLS array_partition variable = msg_metadata.dst_port type              \
    = complete                                               dim = 1
#pragma HLS array_partition variable = msg_metadata.rtps_data type             \
    = complete                                                dim = 1

    uint8_t tx_buf[TX_BUF_LEN]
#ifdef PUB_DATA_FF
    /* Cyber array=EXPAND, array_index=const */
#endif // PUB_DATA_FF
        ;
#ifdef PUB_DATA_FF
#pragma HLS array_partition variable = tx_buf complete dim = 0
#endif // PUB_DATA_FF
#ifdef PUB_DATA_RAM
#pragma HLS bind_storage variable = tx_buf type = ram_t2p
#endif // PUB_DATA_RAM

    uint16_t tx_buf_len;

    if (!in.read_nb(msg_metadata)) {
        return;
    }

    if (msg_metadata.message_type == MSG_TYPE_SPDP) {
        tx_buf_len = spdp_writer_out(conf, &msg_metadata, tx_buf);
    }

    ip_set_checksum(tx_buf);
    udp_set_checksum(tx_buf);

#ifndef USEF_FIFOIF_ETHERNET
    hls_stream<hls_uint<9>> s /* Cyber fifo_size=2 */;
#pragma HLS stream variable = s depth = 2
#endif // !USE_FIFOIF_ETHERNET

    uint16_t i = 0;
    while (i < tx_buf_len) {
        if (!out.full()) {
#ifdef USE_FIFOIF_ETHERNET
            out.write(tx_buf[i]);
#else  // !USE_FIFOIF_ETHERNET
            hls_uint<9> x = tx_buf[i];
            if (i == (tx_buf_len - 1)) {
                x |= 0x100;
            }
            s.write(x);
            slip_out(s, out);
#endif // USE_FIFOIF_ETHERNET
            i++;
        }
    }
}
