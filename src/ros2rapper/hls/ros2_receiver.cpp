#include "ros2_receiver.hpp"
#include "app.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include "remove_endpoints.hpp"
#include "ros2.hpp"
#include "sedp.hpp"
#include "slip.hpp"
#include "spdp.hpp"
#include "udp.hpp"
#include <cstdint>

#define USE_FIFOIF_ETHERNET

#ifdef USE_FIFOIF_ETHERNET
/* Cyber func=inline */
void pre_ip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out) {
#pragma HLS inline
    static uint16_t offset = 0;
    static uint16_t len = 0;

    uint8_t x = in.read();

    switch (offset) {
    case IP_HDR_OFFSET_TOT_LEN:
        len = (uint16_t)x << 8;
        break;
    case IP_HDR_OFFSET_TOT_LEN + 1:
        len |= (uint16_t)x;
    }

    offset++;
    if (offset == len) {
        out.write(x | 0x100);
        offset = 0;
        len = 0;
    } else {
        out.write(x);
    }
};
#endif // USE_FIFOIF_ETHERNET

/* Cyber func=process, bdltran_option=-s, process_valid=NO */
void ros2_receiver(
    hls_stream<uint8_t>     &in /* Cyber port_mode=cw_fifo */,
    hls_stream<rtps_data_t> &out /* Cyber port_mode=axi_stream */,
    uint32_t rawudp_rxbuf[RAWUDP_RXBUF_LEN / 4] /* Cyber mem_reg=1 */,
    uint8_t  ip_payloads[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS],
    hls_uint<PUB_TOPICS_MAX> pub_enable /* Cyber port_mode=in */,
    hls_uint<SUB_TOPICS_MAX> sub_enable /* Cyber port_mode=in */,
    const receiver_config_t *conf /* Cyber port_mode=in, stable_input */,
    VOLATILE                 hls_uint<SUB_TOPICS_MAX>
            *sub_app_data_recv /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE hls_uint<SUB_TOPICS_MAX>
            *sub_app_data_req /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE hls_uint<SUB_TOPICS_MAX>
            *sub_app_data_rel /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE hls_uint<SUB_TOPICS_MAX>
            *sub_app_data_grant /* Cyber port_mode=shared, volatile=YES */,
    uint8_t  sub_app_data_0
        [MAX_APP_DATA_LEN] /* Cyber array=RAM, port_mode=shared, mem_reg=1 */,
    uint8_t sub_app_data_1
        [MAX_APP_DATA_LEN] /* Cyber array=RAM, port_mode=shared, mem_reg=1 */,
    uint8_t sub_app_data_2
        [MAX_APP_DATA_LEN] /* Cyber array=RAM, port_mode=shared, mem_reg=1 */,
    uint8_t sub_app_data_3
        [MAX_APP_DATA_LEN] /* Cyber array=RAM, port_mode=shared, mem_reg=1 */,
    VOLATILE app_data_len_t
        sub_app_data_len[SUB_TOPICS_MAX] /* Cyber array=EXPAND,
                                            port_mode=shared, volatile=yes */
    ,
    VOLATILE uint16_t
        sub_app_data_rep_id[SUB_TOPICS_MAX] /* Cyber array=EXPAND,
                                               port_mode=shared, volatile=yes */
    ,
    VOLATILE uint8_t
        *rawudp_rxbuf_rel /* Cyber port_mode=shared, volatile=YES */,
    VOLATILE uint8_t
                *rawudp_rxbuf_grant /* Cyber port_mode=shared, volatile=YES */,
    hls_uint<9> *xout) {
#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = ap_fifo port = in
#pragma HLS interface mode = axis port = out
#pragma HLS interface mode = ap_memory port = rawudp_rxbuf
#pragma HLS interface mode = ap_memory port = ip_payloads storage_type = ram_1p
#pragma HLS interface mode = ap_none port = pub_enable
#pragma HLS interface mode = ap_none port = sub_enable
#pragma HLS disaggregate             variable = conf
#pragma HLS array_reshape variable = conf->ip_addr type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->ip_addr
#pragma HLS array_reshape variable = conf->subnet_mask type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->subnet_mask
#pragma HLS array_reshape variable = conf->rx_udp_port type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->rx_udp_port
#pragma HLS interface mode = ap_none port = conf->port_num_seed
#pragma HLS interface mode = ap_none port = conf->fragment_expiration
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
#pragma HLS interface mode = ap_none port = conf->ignore_ip_checksum
#pragma HLS interface mode = ap_memory port = sub_app_data_0
#pragma HLS interface mode = ap_memory port = sub_app_data_1
#pragma HLS interface mode = ap_memory port = sub_app_data_2
#pragma HLS interface mode = ap_memory port = sub_app_data_3
#pragma HLS array_partition variable = sub_app_data_len type = complete dim = 1
#pragma HLS interface mode = ap_vld port = sub_app_data_len
#pragma HLS array_partition variable = sub_app_data_rep_id type = complete dim \
    = 1
#pragma HLS interface mode = ap_vld port = sub_app_data_rep_id
#pragma HLS interface mode = ap_vld port = sub_app_data_recv
#pragma HLS interface mode = ap_vld port = sub_app_data_req
#pragma HLS interface mode = ap_vld port = sub_app_data_rel
#pragma HLS interface mode = ap_ack port = sub_app_data_grant
#pragma HLS interface mode = ap_vld port = rawudp_rxbuf_rel
#pragma HLS interface mode = ap_ack port = rawudp_rxbuf_grant

    static bool ip_parity_error = false;
    static bool udp_parity_error = false;

    static const uint8_t app_reader_entity_id_list[SUB_TOPICS_MAX]
                                                  [4] /* Cyber array=EXPAND */
        = ENTITYID_APP_READER_LIST;
#pragma HLS array_partition variable = app_reader_entity_id_list complete dim  \
    = 0

#pragma HLS inline
    static hls_stream<hls_uint<9>> s1 /* Cyber fifo_size=2 */;
    static hls_stream<hls_uint<9>> s2 /* Cyber fifo_size=2 */;
    static hls_stream<hls_uint<9>> s3 /* Cyber fifo_size=2 */;
#pragma HLS stream variable = s1 depth = 2
#pragma HLS stream variable = s2 depth = 2
#pragma HLS stream variable = s3 depth = 2

    hls_uint<1> enable = (pub_enable != 0) || (sub_enable != 0);

    hls_uint<9> x;

#ifdef USE_FIFOIF_ETHERNET
    pre_ip_in(in, s1);
#else  // !USE_FIFOIF_ETHERNET
    slip_in(in, s1);
#endif // USE_FIFOIF_ETHERNET
    ip_in(s1, s2, ip_payloads, conf->fragment_expiration,
          conf->ignore_ip_checksum, ip_parity_error);
    udp_in(s2, s3, enable, conf->rx_udp_port, rawudp_rxbuf, rawudp_rxbuf_rel,
           rawudp_rxbuf_grant, udp_parity_error);

    if (!s3.read_nb(x))
        return;

    update_liveliness(x, out, conf->guid_prefix);

    spdp_reader(x, out, enable, conf->ip_addr, conf->subnet_mask,
                conf->port_num_seed);

    sedp_reader(x, out, pub_enable, sub_enable, conf->ip_addr,
                conf->subnet_mask, conf->port_num_seed, conf->guid_prefix,
                conf->pub_topic_name, conf->pub_topic_name_len,
                conf->pub_topic_type_name, conf->pub_topic_type_name_len,
                conf->sub_topic_name, conf->sub_topic_name_len,
                conf->sub_topic_type_name, conf->sub_topic_type_name_len);

    if (sub_enable != 0) {
        app_reader(x, conf->guid_prefix, app_reader_entity_id_list, sub_enable,
                   sub_app_data_recv, sub_app_data_req, sub_app_data_rel,
                   sub_app_data_grant, sub_app_data_0, sub_app_data_1,
                   sub_app_data_2, sub_app_data_3, sub_app_data_len,
                   sub_app_data_rep_id);
    }

    *xout = x; // Workaround for CWB: FIFO read request will not be
               // asserted if result of read_nb() is unused.
}
