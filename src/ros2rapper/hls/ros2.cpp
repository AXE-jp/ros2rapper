// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "common.hpp"

#include "app.hpp"
#include "endpoint.hpp"
#include "ip.hpp"
#include "ros2.hpp"
#include "sedp.hpp"
#include "slip.hpp"
#include "spdp.hpp"
#include "udp.hpp"

#define USE_FIFOIF_ETHERNET

class tx_buf {
  public:
    uint16_t head;
    uint16_t len;
    uint8_t  buf[TX_BUF_LEN] /* Cyber array=EXPAND,array_index=const */;

    /* Cyber func=inline */
    uint8_t deque() {
#pragma HLS inline
#pragma HLS array_partition variable = buf complete dim = 0
        return buf[head++];
    }

    /* Cyber func=inline */
    bool empty() const {
#pragma HLS inline
        return head == len;
    }

#ifdef USE_FIFOIF_ETHERNET
    /* Cyber func=inline */
    void shift_out(hls_stream<uint8_t> &out) {
#pragma HLS inline
        if (!empty() && !out.full())
            out.write(deque());
    }
#else // !USE_FIFOIF_ETHERNET
    /* Cyber func=inline */
    void shift_out(hls_stream<hls_uint<9>> &out) {
#pragma HLS inline
        if (!empty() && !out.full()) {
            uint8_t data = deque();
            bool    end = empty();

            out.write(data | (end ? 0x100 : 0));
        }
    }
#endif // USE_FIFOIF_ETHERNET
};

#ifdef USE_FIFOIF_ETHERNET
/* Cyber func=inline */
void pre_ip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out) {
#pragma HLS inline
    static uint16_t offset = 0;
    static uint16_t len = 0;

    uint8_t x;

    if (out.full())
        return;

    if (!in.read_nb(x))
        return;

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

/* Cyber func=inline */
static void ros2_in(
    hls_stream<uint8_t> &in, uint32_t rawudp_rxbuf[],
    uint8_t ip_payloads[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS],
    sedp_reader_id_t &sedp_reader_cnt,
    sedp_endpoint     sedp_reader_tbl[SEDP_READER_MAX],
    app_reader_id_t  &app_reader_cnt,
    app_endpoint app_reader_tbl[APP_READER_MAX], hls_uint<1> pub_enable,
    hls_uint<1> sub_enable, const config_t *conf,
    volatile uint8_t *sub_app_data_recv, volatile uint8_t *sub_app_data_req,
    volatile uint8_t *sub_app_data_rel, volatile uint8_t *sub_app_data_grant,
    uint8_t sub_app_data[MAX_APP_DATA_LEN], volatile uint8_t *sub_app_data_len,
    volatile uint16_t *sub_app_data_rep_id, volatile uint8_t *rawudp_rxbuf_rel,
    volatile uint8_t *rawudp_rxbuf_grant, bool ignore_ip_checksum,
    hls_uint<9> *xout) {
    static bool ip_parity_error = false;
    static bool udp_parity_error = false;

    static const uint8_t app_reader_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_APP_READER;
#pragma HLS array_partition variable = app_reader_entity_id complete dim = 0

#pragma HLS inline
    static hls_stream<hls_uint<9>> s1 /* Cyber fifo_size=2 */;
    static hls_stream<hls_uint<9>> s2 /* Cyber fifo_size=2 */;
    static hls_stream<hls_uint<9>> s3 /* Cyber fifo_size=2 */;
#pragma HLS stream variable = s1 depth = 2
#pragma HLS stream variable = s2 depth = 2
#pragma HLS stream variable = s3 depth = 2

    hls_uint<1> enable = pub_enable | sub_enable;

    hls_uint<9> x;

#ifdef USE_FIFOIF_ETHERNET
    pre_ip_in(in, s1);
#else  // !USE_FIFOIF_ETHERNET
    slip_in(in, s1);
#endif // USE_FIFOIF_ETHERNET
    ip_in(s1, s2, ip_payloads, conf->fragment_expiration, ignore_ip_checksum,
          ip_parity_error);
    udp_in(s2, s3, enable, conf->rx_udp_port, rawudp_rxbuf, rawudp_rxbuf_rel,
           rawudp_rxbuf_grant, udp_parity_error);

    if (!s3.read_nb(x))
        return;

    spdp_reader(x, sedp_reader_cnt, sedp_reader_tbl, enable, conf->ip_addr,
                conf->subnet_mask, conf->port_num_seed);

    sedp_reader(x, sedp_reader_tbl, app_reader_cnt, app_reader_tbl, enable,
                conf->ip_addr, conf->subnet_mask, conf->port_num_seed,
                conf->guid_prefix, conf->pub_topic_name,
                conf->pub_topic_name_len, conf->pub_topic_type_name,
                conf->pub_topic_type_name_len, conf->sub_topic_name,
                conf->sub_topic_name_len, conf->sub_topic_type_name,
                conf->sub_topic_type_name_len);

    if (sub_enable) {
        app_reader(x, conf->guid_prefix, app_reader_entity_id,
                   sub_app_data_recv, sub_app_data_req, sub_app_data_rel,
                   sub_app_data_grant, sub_app_data, sub_app_data_len,
                   sub_app_data_rep_id);
    }

    *xout = x; // Workaround for CWB: FIFO read request will not be
               // asserted if result of read_nb() is unused.
}

/* Cyber func=inline */
static void spdp_writer_out(const uint8_t metatraffic_port[2],
                            const uint8_t default_port[2], tx_buf &tx_buf,
                            const config_t *conf) {
    static const uint8_t dst_addr[4] /* Cyber array=EXPAND */
        = IP_MULTICAST_ADDR;
    uint8_t dst_port[2] /* Cyber array=EXPAND */;
    dst_port[0] = DISCOVERY_TRAFFIC_MULTICAST_PORT_0(conf->port_num_seed);
    dst_port[1] = DISCOVERY_TRAFFIC_MULTICAST_PORT_1(conf->port_num_seed);
#pragma HLS array_partition variable = dst_addr complete dim = 0
#pragma HLS array_partition variable = dst_port complete dim = 0

    ip_set_header(conf->ip_addr, dst_addr, IP_HDR_TTL_MULTICAST,
                  SPDP_WRITER_UDP_PKT_LEN, tx_buf.buf);

    udp_set_header(conf->node_udp_port, dst_port, SPDP_WRITER_RTPS_PKT_LEN,
                   tx_buf.buf + IP_HDR_SIZE);

    spdp_writer(conf->guid_prefix, conf->ip_addr, metatraffic_port,
                conf->ip_addr, default_port,
                tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE), conf->node_name,
                conf->node_name_len);

    tx_buf.head = 0;
    tx_buf.len = SPDP_WRITER_IP_PKT_LEN;
}

/* Cyber func=inline */
static void sedp_pub_writer_out(
    const uint8_t writer_entity_id[4], const uint8_t dst_addr[4],
    const uint8_t dst_port[2], const uint8_t reader_guid_prefix[12],
    const uint8_t reader_entity_id[4], const uint8_t usertraffic_port[2],
    const uint8_t app_entity_id[4], tx_buf &tx_buf, const config_t *conf) {
    ip_set_header(conf->ip_addr, dst_addr, IP_HDR_TTL_UNICAST,
                  SEDP_WRITER_UDP_PKT_LEN, tx_buf.buf);

    udp_set_header(conf->node_udp_port, dst_port, SEDP_WRITER_RTPS_PKT_LEN,
                   tx_buf.buf + IP_HDR_SIZE);

    sedp_writer(conf->guid_prefix, writer_entity_id, reader_guid_prefix,
                reader_entity_id, conf->ip_addr, usertraffic_port,
                app_entity_id, tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE),
                conf->pub_topic_name, conf->pub_topic_name_len,
                conf->pub_topic_type_name, conf->pub_topic_type_name_len);

    tx_buf.head = 0;
    tx_buf.len = SEDP_WRITER_IP_PKT_LEN;
}

/* Cyber func=inline */
static void sedp_sub_writer_out(
    const uint8_t writer_entity_id[4], const uint8_t dst_addr[4],
    const uint8_t dst_port[2], const uint8_t reader_guid_prefix[12],
    const uint8_t reader_entity_id[4], const uint8_t usertraffic_port[2],
    const uint8_t app_entity_id[4], tx_buf &tx_buf, const config_t *conf) {
    ip_set_header(conf->ip_addr, dst_addr, IP_HDR_TTL_UNICAST,
                  SEDP_WRITER_UDP_PKT_LEN, tx_buf.buf);

    udp_set_header(conf->node_udp_port, dst_port, SEDP_WRITER_RTPS_PKT_LEN,
                   tx_buf.buf + IP_HDR_SIZE);

    sedp_writer(conf->guid_prefix, writer_entity_id, reader_guid_prefix,
                reader_entity_id, conf->ip_addr, usertraffic_port,
                app_entity_id, tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE),
                conf->sub_topic_name, conf->sub_topic_name_len,
                conf->sub_topic_type_name, conf->sub_topic_type_name_len);

    tx_buf.head = 0;
    tx_buf.len = SEDP_WRITER_IP_PKT_LEN;
}

/* Cyber func=inline */
static void sedp_heartbeat_out(
    const uint8_t writer_entity_id[4], const uint8_t dst_addr[4],
    const uint8_t dst_port[2], const uint8_t reader_guid_prefix[12],
    const uint8_t reader_entity_id[4], const int64_t first_seqnum,
    const int64_t last_seqnum, tx_buf &tx_buf, uint32_t &cnt,
    const uint8_t src_addr[4], const uint8_t src_port[2],
    const uint8_t writer_guid_prefix[12]) {
    cnt++;

    ip_set_header(src_addr, dst_addr, IP_HDR_TTL_UNICAST,
                  SEDP_HEARTBEAT_UDP_PKT_LEN, tx_buf.buf);

    udp_set_header(src_port, dst_port, SEDP_HEARTBEAT_RTPS_PKT_LEN,
                   tx_buf.buf + IP_HDR_SIZE);

    sedp_heartbeat(writer_guid_prefix, writer_entity_id, reader_guid_prefix,
                   reader_entity_id, first_seqnum, last_seqnum, cnt,
                   tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE));

    tx_buf.head = 0;
    tx_buf.len = SEDP_HEARTBEAT_IP_PKT_LEN;
}

/* Cyber func=inline */
static void
sedp_acknack_out(const uint8_t writer_entity_id[4], const uint8_t dst_addr[4],
                 const uint8_t dst_port[2],
                 const uint8_t reader_guid_prefix[12],
                 const uint8_t reader_entity_id[4], uint8_t snstate_base,
                 bool snstate_is_empty, tx_buf &tx_buf, uint32_t &cnt,
                 const uint8_t src_addr[4], const uint8_t src_port[2],
                 const uint8_t writer_guid_prefix[12]) {
    cnt++;

    ip_set_header(src_addr, dst_addr, IP_HDR_TTL_UNICAST,
                  SEDP_ACKNACK_UDP_PKT_LEN, tx_buf.buf);

    udp_set_header(src_port, dst_port, SEDP_ACKNACK_RTPS_PKT_LEN,
                   tx_buf.buf + IP_HDR_SIZE);

    sedp_acknack(writer_guid_prefix, writer_entity_id, reader_guid_prefix,
                 reader_entity_id, snstate_base, snstate_is_empty, cnt,
                 tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE));

    tx_buf.head = 0;
    tx_buf.len = SEDP_ACKNACK_IP_PKT_LEN;
}

/* Cyber func=inline */
static void
app_writer_out(const uint8_t writer_entity_id[4], const uint8_t dst_addr[4],
               const uint8_t dst_port[2], const uint8_t reader_guid_prefix[12],
               const uint8_t reader_entity_id[4], tx_buf &tx_buf,
               int64_t &seqnum, const uint8_t src_addr[4],
               const uint8_t src_port[2], const uint8_t writer_guid_prefix[12],
               volatile const uint8_t  pub_app_data[MAX_APP_DATA_LEN],
               volatile const uint8_t *pub_app_data_len) {
    seqnum++;

    ip_set_header(src_addr, dst_addr, IP_HDR_TTL_UNICAST,
                  APP_WRITER_UDP_PKT_LEN(*pub_app_data_len), tx_buf.buf);

    udp_set_header(src_port, dst_port,
                   APP_WRITER_RTPS_PKT_LEN(*pub_app_data_len),
                   tx_buf.buf + IP_HDR_SIZE);

    app_writer(writer_guid_prefix, writer_entity_id, reader_guid_prefix,
               reader_entity_id, seqnum, pub_app_data, *pub_app_data_len,
               tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE));

    tx_buf.head = 0;
    tx_buf.len = APP_WRITER_IP_PKT_LEN(*pub_app_data_len);
}

/* Cyber func=inline */
static void rawudp_out(const uint8_t dst_addr[4], const uint8_t dst_port[2],
                       tx_buf &tx_buf, const uint8_t src_addr[4],
                       const uint8_t src_port[2], uint8_t udp_payload_len) {
    ip_set_header(src_addr, dst_addr, IP_HDR_TTL_UNICAST,
                  udp_payload_len + UDP_HDR_SIZE, tx_buf.buf);

    udp_set_header(src_port, dst_port, udp_payload_len,
                   tx_buf.buf + IP_HDR_SIZE);

    tx_buf.head = 0;
    tx_buf.len = IP_HDR_SIZE + UDP_HDR_SIZE + udp_payload_len;
}

#define SPDP_WRITER_OUT()                                                      \
    do {                                                                       \
        spdp_writer_out(metatraffic_port, default_port, tx_buf, conf);         \
    } while (0)

#define SEDP_PUB_WRITER_OUT(id)                                                \
    do {                                                                       \
        if (sedp_reader_cnt > (id)) {                                          \
            sedp_pub_writer_out(                                               \
                pub_writer_entity_id, sedp_reader_tbl[(id)].ip_addr,           \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix, pub_reader_entity_id,       \
                default_port, app_writer_entity_id, tx_buf, conf);             \
        }                                                                      \
    } while (0)

#define SEDP_SUB_WRITER_OUT(id)                                                \
    do {                                                                       \
        if (sedp_reader_cnt > (id)) {                                          \
            sedp_sub_writer_out(                                               \
                sub_writer_entity_id, sedp_reader_tbl[(id)].ip_addr,           \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix, sub_reader_entity_id,       \
                default_port, app_reader_entity_id, tx_buf, conf);             \
        }                                                                      \
    } while (0)

#define SEDP_PUB_HEARTBEAT_OUT(id, pub_enable)                                 \
    do {                                                                       \
        if (sedp_reader_cnt > (id)) {                                          \
            sedp_heartbeat_out(                                                \
                pub_writer_entity_id, sedp_reader_tbl[(id)].ip_addr,           \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix, pub_reader_entity_id, 1,    \
                (pub_enable) ? 1 : 0, tx_buf, sedp_pub_heartbeat_cnt[(id)],    \
                conf->ip_addr, conf->node_udp_port, conf->guid_prefix);        \
        }                                                                      \
    } while (0)

#define SEDP_SUB_HEARTBEAT_OUT(id, sub_enable)                                 \
    do {                                                                       \
        if (sedp_reader_cnt > (id)) {                                          \
            sedp_heartbeat_out(                                                \
                sub_writer_entity_id, sedp_reader_tbl[(id)].ip_addr,           \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix, sub_reader_entity_id, 1,    \
                (sub_enable) ? 1 : 0, tx_buf, sedp_sub_heartbeat_cnt[(id)],    \
                conf->ip_addr, conf->node_udp_port, conf->guid_prefix);        \
            if (sedp_reader_tbl[(id)].initial_send_counter != 3)               \
                sedp_reader_tbl[(id)].initial_send_counter++;                  \
        }                                                                      \
    } while (0)

#define SEDP_PUB_ACKNACK_OUT(id)                                               \
    do {                                                                       \
        if (sedp_reader_cnt > (id)) {                                          \
            sedp_acknack_out(                                                  \
                pub_writer_entity_id, sedp_reader_tbl[(id)].ip_addr,           \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix, pub_reader_entity_id,       \
                snstate_base, snstate_is_empty, tx_buf,                        \
                sedp_pub_acknack_cnt[(id)], conf->ip_addr,                     \
                conf->node_udp_port, conf->guid_prefix);                       \
            sedp_reader_tbl[(id)].builtin_pubrd_acknack_req = false;           \
        }                                                                      \
    } while (0)

#define SEDP_SUB_ACKNACK_OUT(id)                                               \
    do {                                                                       \
        if (sedp_reader_cnt > (id)) {                                          \
            sedp_acknack_out(                                                  \
                sub_writer_entity_id, sedp_reader_tbl[(id)].ip_addr,           \
                sedp_reader_tbl[(id)].udp_port,                                \
                sedp_reader_tbl[(id)].guid_prefix, sub_reader_entity_id,       \
                snstate_base, snstate_is_empty, tx_buf,                        \
                sedp_sub_acknack_cnt[(id)], conf->ip_addr,                     \
                conf->node_udp_port, conf->guid_prefix);                       \
            sedp_reader_tbl[(id)].builtin_subrd_acknack_req = false;           \
        }                                                                      \
    } while (0)

/* Cyber func=inline */
void APP_WRITER_OUT(app_reader_id_t id, app_reader_id_t app_reader_cnt,
                    app_endpoint            app_reader_tbl[APP_READER_MAX],
                    const config_t         *conf,
                    volatile const uint8_t  pub_app_data[MAX_APP_DATA_LEN],
                    volatile const uint8_t *pub_app_data_len,
                    volatile uint8_t       *pub_app_data_req,
                    volatile uint8_t       *pub_app_data_rel,
                    volatile uint8_t       *pub_app_data_grant,
                    const uint8_t app_writer_entity_id[4], tx_buf &tx_buf,
                    int64_t app_seqnum) {
#pragma HLS inline

    if (app_reader_cnt > id && (app_reader_tbl[id].app_ep_type & APP_EP_PUB)) {

#ifdef VITIS_HLS
    app_data_request_section : {
#pragma HLS protocol fixed
        *pub_app_data_req = 0 /* write dummy value to assert valid signal */;
        ap_wait();
        ap_wait();
    }
#else
        /* Cyber scheduling_block = non-transparent */
        {
            *pub_app_data_req
                = 0 /* write dummy value to assert valid signal */;
            cwb::cwb_clk();
            cwb::cwb_clk();
        }
#endif

        if (*pub_app_data_grant == 1) {
            app_writer_out(app_writer_entity_id, app_reader_tbl[id].ip_addr,
                           app_reader_tbl[id].udp_port,
                           app_reader_tbl[id].guid_prefix,
                           app_reader_tbl[id].entity_id, tx_buf, app_seqnum,
                           conf->ip_addr, conf->node_udp_port,
                           conf->guid_prefix, pub_app_data, pub_app_data_len);

#ifdef VITIS_HLS
        app_data_release_section : {
#pragma HLS protocol fixed
            ap_wait();
            *pub_app_data_rel
                = 0 /* write dummy value to assert valid signal */;
            ap_wait();
            ap_wait();
        }
#else
            /* Cyber scheduling_block = non-transparent */
            {
                cwb::cwb_clk();
                *pub_app_data_rel
                    = 0 /* write dummy value to assert valid signal */;
                cwb::cwb_clk();
                cwb::cwb_clk();
            }
#endif
        }
    }
}

#define RAWUDP_OUT()                                                           \
    do {                                                                       \
        rawudp_out(rawudp_tx_dst_addr, rawudp_tx_dst_port, tx_buf,             \
                   conf->ip_addr, rawudp_tx_src_port, rawudp_tx_payload_len);  \
    } while (0)

/* Cyber func=inline */
static void ros2_out(
    hls_stream<uint8_t> &out, uint32_t rawudp_txbuf[],
    sedp_reader_id_t &sedp_reader_cnt,
    sedp_endpoint     sedp_reader_tbl[SEDP_READER_MAX],
    app_reader_id_t  &app_reader_cnt,
    app_endpoint app_reader_tbl[APP_READER_MAX], hls_uint<1> pub_enable,
    hls_uint<1> sub_enable, const config_t *conf,
    volatile const uint8_t  pub_app_data[MAX_APP_DATA_LEN],
    volatile const uint8_t *pub_app_data_len,
    volatile uint8_t *pub_app_data_req, volatile uint8_t *pub_app_data_rel,
    volatile uint8_t *pub_app_data_grant, volatile uint8_t *rawudp_txbuf_rel,
    volatile uint8_t *rawudp_txbuf_grant, hls_uint<1> cnt_interval_elapsed,
    volatile uint8_t *cnt_interval_set, hls_uint<1> cnt_spdp_wr_elapsed,
    volatile uint8_t *cnt_spdp_wr_set, hls_uint<1> cnt_sedp_pub_wr_elapsed,
    volatile uint8_t *cnt_sedp_pub_wr_set, hls_uint<1> cnt_sedp_sub_wr_elapsed,
    volatile uint8_t *cnt_sedp_sub_wr_set, hls_uint<1> cnt_sedp_pub_hb_elapsed,
    volatile uint8_t *cnt_sedp_pub_hb_set, hls_uint<1> cnt_sedp_sub_hb_elapsed,
    volatile uint8_t *cnt_sedp_sub_hb_set, hls_uint<1> cnt_sedp_pub_an_elapsed,
    volatile uint8_t *cnt_sedp_pub_an_set, hls_uint<1> cnt_sedp_sub_an_elapsed,
    volatile uint8_t *cnt_sedp_sub_an_set, hls_uint<1> cnt_app_wr_elapsed,
    volatile uint8_t *cnt_app_wr_set) {

    static const uint8_t pub_writer_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_WRITER;
    static const uint8_t sub_writer_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_WRITER;
    static const uint8_t app_writer_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_APP_WRITER;
#pragma HLS array_partition variable = pub_writer_entity_id complete dim = 0
#pragma HLS array_partition variable = sub_writer_entity_id complete dim = 0
#pragma HLS array_partition variable = app_writer_entity_id complete dim = 0

    static const uint8_t pub_reader_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PUBLICATIONS_READER;
    static const uint8_t sub_reader_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
    static const uint8_t app_reader_entity_id[4] /* Cyber array=EXPAND */
        = ENTITYID_APP_READER;
#pragma HLS array_partition variable = pub_reader_entity_id complete dim = 0
#pragma HLS array_partition variable = sub_reader_entity_id complete dim = 0
#pragma HLS array_partition variable = app_reader_entity_id complete dim = 0

    uint8_t metatraffic_port[2] /* Cyber array=EXPAND */;
    metatraffic_port[0] = DISCOVERY_TRAFFIC_UNICAST_PORT_0(
        conf->port_num_seed, TARGET_PARTICIPANT_ID);
    metatraffic_port[1] = DISCOVERY_TRAFFIC_UNICAST_PORT_1(
        conf->port_num_seed, TARGET_PARTICIPANT_ID);

    uint8_t default_port[2] /* Cyber array=EXPAND */;
    default_port[0] = USER_TRAFFIC_UNICAST_PORT_0(conf->port_num_seed,
                                                  TARGET_PARTICIPANT_ID);
    default_port[1] = USER_TRAFFIC_UNICAST_PORT_1(conf->port_num_seed,
                                                  TARGET_PARTICIPANT_ID);

#pragma HLS array_partition variable = metatraffic_port complete dim = 0
#pragma HLS array_partition variable = default_port complete dim = 0

    static tx_buf tx_buf;

    static uint32_t
        sedp_pub_heartbeat_cnt[SEDP_READER_MAX] /* Cyber array=EXPAND */;
    static uint32_t
        sedp_sub_heartbeat_cnt[SEDP_READER_MAX] /* Cyber array=EXPAND */;
    static uint32_t
        sedp_pub_acknack_cnt[SEDP_READER_MAX] /* Cyber array=EXPAND */;
    static uint32_t
        sedp_sub_acknack_cnt[SEDP_READER_MAX] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = sedp_pub_heartbeat_cnt complete dim = 0
#pragma HLS array_partition variable = sedp_sub_heartbeat_cnt complete dim = 0
#pragma HLS array_partition variable = sedp_pub_acknack_cnt complete dim = 0
#pragma HLS array_partition variable = sedp_sub_acknack_cnt complete dim = 0

    static int64_t app_seqnum;

#ifndef USE_FIFOIF_ETHERNET
    static hls_stream<hls_uint<9>> s /* Cyber fifo_size=2 */;
#pragma HLS stream variable = s depth = 2
#endif // !USE_FIFOIF_ETHERNET

    static uint8_t rawudp_tx_dst_addr[4] /* Cyber array=EXPAND */;
    static uint8_t rawudp_tx_dst_port[2] /* Cyber array=EXPAND */;
    static uint8_t rawudp_tx_src_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = rawudp_tx_dst_addr complete dim = 0
#pragma HLS array_partition variable = rawudp_tx_dst_port complete dim = 0
#pragma HLS array_partition variable = rawudp_tx_src_port complete dim = 0
    static uint16_t rawudp_tx_payload_len;
    uint32_t        ram_read_buf;

#define RAWUDP_TXBUF_COPY_INIT    0
#define RAWUDP_TXBUF_COPY_RUNNING 1
#define RAWUDP_TXBUF_COPY_DONE    2
    static hls_uint<2> rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_INIT;
    static uint16_t    rawudp_txbuf_rd_off;
    static uint16_t    rawudp_txpayload_wr_off;

    static hls_uint<2> tx_progress;
    static hls_uint<3> tx_cnt_elapsed;

    static hls_uint<3> next_packet_type = 0;
#define ROTATE_NEXT_PACKET_TYPE next_packet_type++

    if (!tx_buf.empty()) {
#ifdef USE_FIFOIF_ETHERNET
        tx_buf.shift_out(out);
#else  // !USE_FIFOIF_ETHERNET
        tx_buf.shift_out(s);

        slip_out(s, out);
#endif // USE_FIFOIF_ETHERNET

        if (tx_buf.empty()) {
            *cnt_interval_set = 1;
        }
    } else {
        if (*rawudp_txbuf_grant == 1) {
            switch (rawudp_txbuf_copy_status) {
            case RAWUDP_TXBUF_COPY_INIT:
                rawudp_txbuf_rd_off = 0;
                rawudp_txpayload_wr_off = 0;
                rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_RUNNING;

                // Clear tx buffer before constructing packet to prevent
                // variable length zero clear.

                /* Cyber unroll_times=all */
                for (int i = 0; i < TX_BUF_LEN; i++) {
#pragma HLS unroll
                    tx_buf.buf[i] = 0;
                }
                break;
            case RAWUDP_TXBUF_COPY_RUNNING:
                switch (rawudp_txbuf_rd_off) {
                case 0:
                    ram_read_buf = rawudp_txbuf[rawudp_txbuf_rd_off];
                    rawudp_tx_dst_addr[0] = ram_read_buf & 0xff;
                    rawudp_tx_dst_addr[1] = (ram_read_buf >> 8) & 0xff;
                    rawudp_tx_dst_addr[2] = (ram_read_buf >> 16) & 0xff;
                    rawudp_tx_dst_addr[3] = (ram_read_buf >> 24) & 0xff;
                    break;
                case 1:
                    ram_read_buf = rawudp_txbuf[rawudp_txbuf_rd_off];
                    rawudp_tx_dst_port[1] = ram_read_buf & 0xff;
                    rawudp_tx_dst_port[0] = (ram_read_buf >> 8) & 0xff;
                    rawudp_tx_src_port[1] = (ram_read_buf >> 16) & 0xff;
                    rawudp_tx_src_port[0] = (ram_read_buf >> 24) & 0xff;
                    break;
                case 2:
                    ram_read_buf = rawudp_txbuf[rawudp_txbuf_rd_off];
                    rawudp_tx_payload_len = ram_read_buf & 0xff;
                    rawudp_tx_payload_len |= (ram_read_buf >> 8) & 0xff;
                    // padding 2byte
                    break;
                default:
                    if (rawudp_txpayload_wr_off == MAX_RAWUDP_OUT_PAYLOAD_LEN) {
                        rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_DONE;
                    } else {
                        ram_read_buf = rawudp_txbuf[rawudp_txbuf_rd_off];
                        tx_buf.buf[rawudp_txpayload_wr_off + 0 + IP_HDR_SIZE
                                   + UDP_HDR_SIZE]
                            = (rawudp_tx_payload_len
                               <= rawudp_txpayload_wr_off + 0)
                                  ? 0
                                  : (ram_read_buf & 0xff);
                        tx_buf.buf[rawudp_txpayload_wr_off + 1 + IP_HDR_SIZE
                                   + UDP_HDR_SIZE]
                            = (rawudp_tx_payload_len
                               <= rawudp_txpayload_wr_off + 1)
                                  ? 0
                                  : ((ram_read_buf >> 8) & 0xff);
                        tx_buf.buf[rawudp_txpayload_wr_off + 2 + IP_HDR_SIZE
                                   + UDP_HDR_SIZE]
                            = (rawudp_tx_payload_len
                               <= rawudp_txpayload_wr_off + 2)
                                  ? 0
                                  : ((ram_read_buf >> 16) & 0xff);
                        tx_buf.buf[rawudp_txpayload_wr_off + 3 + IP_HDR_SIZE
                                   + UDP_HDR_SIZE]
                            = (rawudp_tx_payload_len
                               <= rawudp_txpayload_wr_off + 3)
                                  ? 0
                                  : ((ram_read_buf >> 24) & 0xff);
                        rawudp_txpayload_wr_off += 4;
                    }
                    break;
                }
                rawudp_txbuf_rd_off++;
                break;
            case RAWUDP_TXBUF_COPY_DONE:
                RAWUDP_OUT();
                *rawudp_txbuf_rel = 0 /*write dummy value to assert ap_vld*/;
                rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_INIT;
                break;
            default:
                rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_INIT;
                break;
            }
        } else if (cnt_interval_elapsed) {
            if ((pub_enable | sub_enable) && cnt_spdp_wr_elapsed
                && next_packet_type == 0) {
                SPDP_WRITER_OUT();
                *cnt_spdp_wr_set = 1;
                ROTATE_NEXT_PACKET_TYPE;
            } else if (pub_enable && next_packet_type == 1) {
                if (cnt_sedp_pub_wr_elapsed)
                    tx_cnt_elapsed++;

                if (sedp_reader_tbl[tx_progress].initial_send_counter == 3
                    && !cnt_sedp_pub_wr_elapsed) {
                    if (tx_progress == 3) {
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                    }
                } else {
                    switch (tx_progress) {
                    case 0:
                        SEDP_PUB_WRITER_OUT(0);
                        break;
                    case 1:
                        SEDP_PUB_WRITER_OUT(1);
                        break;
                    case 2:
                        SEDP_PUB_WRITER_OUT(2);
                        break;
                    case 3:
                        SEDP_PUB_WRITER_OUT(3);
                        if (tx_cnt_elapsed == 4)
                            *cnt_sedp_pub_wr_set = 1;
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                        break;
                    }
                }
                tx_progress++;
            } else if (sub_enable && next_packet_type == 2) {
                if (cnt_sedp_sub_wr_elapsed)
                    tx_cnt_elapsed++;

                if (sedp_reader_tbl[tx_progress].initial_send_counter == 3
                    && !cnt_sedp_sub_wr_elapsed) {
                    if (tx_progress == 3) {
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                    }
                } else {
                    switch (tx_progress) {
                    case 0:
                        SEDP_SUB_WRITER_OUT(0);
                        break;
                    case 1:
                        SEDP_SUB_WRITER_OUT(1);
                        break;
                    case 2:
                        SEDP_SUB_WRITER_OUT(2);
                        break;
                    case 3:
                        SEDP_SUB_WRITER_OUT(3);
                        if (tx_cnt_elapsed == 4)
                            *cnt_sedp_sub_wr_set = 1;
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                        break;
                    }
                }
                tx_progress++;
            } else if (next_packet_type == 3) {
                if (cnt_sedp_pub_hb_elapsed)
                    tx_cnt_elapsed++;

                if (sedp_reader_tbl[tx_progress].initial_send_counter == 3
                    && !cnt_sedp_pub_hb_elapsed) {
                    if (tx_progress == 3) {
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                    }
                } else {
                    switch (tx_progress) {
                    case 0:
                        SEDP_PUB_HEARTBEAT_OUT(0, pub_enable);
                        break;
                    case 1:
                        SEDP_PUB_HEARTBEAT_OUT(1, pub_enable);
                        break;
                    case 2:
                        SEDP_PUB_HEARTBEAT_OUT(2, pub_enable);
                        break;
                    case 3:
                        SEDP_PUB_HEARTBEAT_OUT(3, pub_enable);
                        if (tx_cnt_elapsed == 4)
                            *cnt_sedp_pub_hb_set = 1;
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                        break;
                    }
                }
                tx_progress++;
            } else if (next_packet_type == 4) {
                if (cnt_sedp_sub_hb_elapsed)
                    tx_cnt_elapsed++;

                if (sedp_reader_tbl[tx_progress].initial_send_counter == 3
                    && !cnt_sedp_sub_hb_elapsed) {
                    if (tx_progress == 3) {
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                    }
                } else {
                    switch (tx_progress) {
                    case 0:
                        SEDP_SUB_HEARTBEAT_OUT(0, sub_enable);
                        break;
                    case 1:
                        SEDP_SUB_HEARTBEAT_OUT(1, sub_enable);
                        break;
                    case 2:
                        SEDP_SUB_HEARTBEAT_OUT(2, sub_enable);
                        break;
                    case 3:
                        SEDP_SUB_HEARTBEAT_OUT(3, sub_enable);
                        if (tx_cnt_elapsed == 4)
                            *cnt_sedp_sub_hb_set = 1;
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                        break;
                    }
                }
                tx_progress++;
            } else if (next_packet_type == 5) {
                uint8_t wr_seqnum
                    = sedp_reader_tbl[tx_progress].builtin_pubrd_wr_seqnum;
                uint8_t rd_seqnum
                    = sedp_reader_tbl[tx_progress].builtin_pubrd_rd_seqnum;
                bool acknack_req
                    = sedp_reader_tbl[tx_progress].builtin_pubrd_acknack_req;
                uint8_t snstate_base = rd_seqnum;
                bool    snstate_is_empty = (wr_seqnum < rd_seqnum);

                if (cnt_sedp_pub_an_elapsed)
                    tx_cnt_elapsed++;

                if (cnt_sedp_pub_an_elapsed
                    || (acknack_req && !snstate_is_empty)) {
                    switch (tx_progress) {
                    case 0:
                        SEDP_PUB_ACKNACK_OUT(0);
                        break;
                    case 1:
                        SEDP_PUB_ACKNACK_OUT(1);
                        break;
                    case 2:
                        SEDP_PUB_ACKNACK_OUT(2);
                        break;
                    case 3:
                        SEDP_PUB_ACKNACK_OUT(3);
                        if (tx_cnt_elapsed == 4)
                            *cnt_sedp_pub_an_set = 1;
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                        break;
                    }
                } else {
                    if (tx_progress == 3) {
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                    }
                }
                tx_progress++;
            } else if (next_packet_type == 6) {
                uint8_t wr_seqnum
                    = sedp_reader_tbl[tx_progress].builtin_subrd_wr_seqnum;
                uint8_t rd_seqnum
                    = sedp_reader_tbl[tx_progress].builtin_subrd_rd_seqnum;
                bool acknack_req
                    = sedp_reader_tbl[tx_progress].builtin_subrd_acknack_req;
                uint8_t snstate_base = rd_seqnum;
                bool    snstate_is_empty = (wr_seqnum < rd_seqnum);

                if (cnt_sedp_sub_an_elapsed)
                    tx_cnt_elapsed++;

                if (cnt_sedp_sub_an_elapsed
                    || (acknack_req && !snstate_is_empty)) {
                    switch (tx_progress) {
                    case 0:
                        SEDP_SUB_ACKNACK_OUT(0);
                        break;
                    case 1:
                        SEDP_SUB_ACKNACK_OUT(1);
                        break;
                    case 2:
                        SEDP_SUB_ACKNACK_OUT(2);
                        break;
                    case 3:
                        SEDP_SUB_ACKNACK_OUT(3);
                        if (tx_cnt_elapsed == 4)
                            *cnt_sedp_sub_an_set = 1;
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                        break;
                    }
                } else {
                    if (tx_progress == 3) {
                        ROTATE_NEXT_PACKET_TYPE;
                        tx_cnt_elapsed = 0;
                    }
                }
                tx_progress++;
            } else if (pub_enable && cnt_app_wr_elapsed
                       && next_packet_type == 7) {
                switch (tx_progress) {
                case 0:
                    APP_WRITER_OUT(0, app_reader_cnt, app_reader_tbl, conf,
                                   pub_app_data, pub_app_data_len,
                                   pub_app_data_req, pub_app_data_rel,
                                   pub_app_data_grant, app_writer_entity_id,
                                   tx_buf, app_seqnum);
                    break;
                case 1:
                    APP_WRITER_OUT(1, app_reader_cnt, app_reader_tbl, conf,
                                   pub_app_data, pub_app_data_len,
                                   pub_app_data_req, pub_app_data_rel,
                                   pub_app_data_grant, app_writer_entity_id,
                                   tx_buf, app_seqnum);
                    break;
                case 2:
                    APP_WRITER_OUT(2, app_reader_cnt, app_reader_tbl, conf,
                                   pub_app_data, pub_app_data_len,
                                   pub_app_data_req, pub_app_data_rel,
                                   pub_app_data_grant, app_writer_entity_id,
                                   tx_buf, app_seqnum);
                    break;
                case 3:
                    APP_WRITER_OUT(3, app_reader_cnt, app_reader_tbl, conf,
                                   pub_app_data, pub_app_data_len,
                                   pub_app_data_req, pub_app_data_rel,
                                   pub_app_data_grant, app_writer_entity_id,
                                   tx_buf, app_seqnum);
                    *cnt_app_wr_set = 1;
                    ROTATE_NEXT_PACKET_TYPE;
                    break;
                }
                tx_progress++;
            } else {
                ROTATE_NEXT_PACKET_TYPE;
            }
        }

        if (!tx_buf.empty()) {
            ip_set_checksum(tx_buf.buf);
            udp_set_checksum(tx_buf.buf);
        }
    }
}

/* Cyber func=process, bdltran_option=-s, process_valid=NO */
void ros2(
    hls_stream<uint8_t> &in /* Cyber port_mode=cw_fifo */,
    hls_stream<uint8_t> &out /* Cyber port_mode=cw_fifo */,
    uint32_t             udp_rxbuf[RAWUDP_RXBUF_LEN / 4],
    uint32_t             udp_txbuf[RAWUDP_TXBUF_LEN / 4],
    uint8_t ip_payloads[MAX_PENDINGS * IP_MAX_PAYLOAD_LEN * MAX_IP_FRAGMENTS],
    hls_uint<1>            pub_enable /* Cyber port_mode=in */,
    hls_uint<1>            sub_enable /* Cyber port_mode=in */,
    const config_t        *conf /* Cyber port_mode=in, stable_input */,
    volatile const uint8_t pub_app_data
        [MAX_APP_DATA_LEN] /* Cyber array=EXPAND, port_mode=shared */,
    volatile const uint8_t *pub_app_data_len /* Cyber port_mode=cw_fifo */,
    uint8_t
        sub_app_data[MAX_APP_DATA_LEN] /* Cyber array=RAM, port_mode=shared */,
    volatile uint8_t *sub_app_data_len, volatile uint16_t *sub_app_data_rep_id,
    volatile uint8_t *pub_app_data_req /* Cyber port_mode=shared */,
    volatile uint8_t *pub_app_data_rel /* Cyber port_mode=shared */,
    volatile uint8_t *pub_app_data_grant /* Cyber port_mode=shared */,
    volatile uint8_t *sub_app_data_recv /* Cyber port_mode=shared */,
    volatile uint8_t *sub_app_data_req /* Cyber port_mode=shared */,
    volatile uint8_t *sub_app_data_rel /* Cyber port_mode=shared */,
    volatile uint8_t *sub_app_data_grant /* Cyber port_mode=shared */,
    volatile uint8_t *udp_rxbuf_rel /* Cyber port_mode=shared */,
    volatile uint8_t *udp_rxbuf_grant /* Cyber port_mode=shared */,
    volatile uint8_t *udp_txbuf_rel /* Cyber port_mode=shared */,
    volatile uint8_t *udp_txbuf_grant /* Cyber port_mode=shared */,

    hls_uint<1> cnt_interval_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_spdp_wr_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_pub_wr_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_sub_wr_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_pub_hb_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_sub_hb_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_pub_an_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_sedp_sub_an_elapsed /* Cyber port_mode=in */,
    hls_uint<1> cnt_app_wr_elapsed /* Cyber port_mode=in */,

    volatile uint8_t *cnt_interval_set /* Cyber port_mode=shared */,
    volatile uint8_t *cnt_spdp_wr_set /* Cyber port_mode=shared */,
    volatile uint8_t *cnt_sedp_pub_wr_set /* Cyber port_mode=shared */,
    volatile uint8_t *cnt_sedp_sub_wr_set /* Cyber port_mode=shared */,
    volatile uint8_t *cnt_sedp_pub_hb_set /* Cyber port_mode=shared */,
    volatile uint8_t *cnt_sedp_sub_hb_set /* Cyber port_mode=shared */,
    volatile uint8_t *cnt_sedp_pub_an_set /* Cyber port_mode=shared */,
    volatile uint8_t *cnt_sedp_sub_an_set /* Cyber port_mode=shared */,
    volatile uint8_t *cnt_app_wr_set /* Cyber port_mode=shared */,
    hls_uint<9>      *xout) {

#pragma HLS interface mode = ap_fifo port = in
#pragma HLS interface mode = ap_fifo port = out
#pragma HLS interface mode = ap_memory port = udp_rxbuf
#pragma HLS interface mode = ap_memory port = udp_txbuf storage_type = ram_1p
#pragma HLS interface mode = ap_memory port = ip_payloads storage_type = ram_1p
#pragma HLS interface mode = ap_none port = pub_enable
#pragma HLS interface mode = ap_none port = sub_enable
#pragma HLS disaggregate             variable = conf
#pragma HLS array_reshape variable = conf->ip_addr type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->ip_addr
#pragma HLS array_reshape variable = conf->subnet_mask type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->subnet_mask
#pragma HLS array_reshape variable = conf->node_name type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->node_name
#pragma HLS interface mode = ap_none port = conf->node_name_len
#pragma HLS array_reshape variable = conf->node_udp_port type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->node_udp_port
#pragma HLS array_reshape variable = conf->rx_udp_port type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->rx_udp_port
#pragma HLS interface mode = ap_none port = conf->port_num_seed
#pragma HLS interface mode = ap_none port = conf->fragment_expiration
#pragma HLS array_reshape variable = conf->guid_prefix type = complete dim = 0
#pragma HLS interface mode = ap_none port = conf->guid_prefix
#pragma HLS array_reshape variable = conf->pub_topic_name type = complete dim  \
    = 0
#pragma HLS interface mode = ap_none port = conf->pub_topic_name
#pragma HLS interface mode = ap_none port = conf->pub_topic_name_len
#pragma HLS array_reshape variable = conf->pub_topic_type_name type            \
    = complete                                                 dim = 0
#pragma HLS interface mode = ap_none port = conf->pub_topic_type_name
#pragma HLS interface mode = ap_none port = conf->pub_topic_type_name_len
#pragma HLS array_reshape variable = conf->sub_topic_name type = complete dim  \
    = 0
#pragma HLS interface mode = ap_none port = conf->sub_topic_name
#pragma HLS interface mode = ap_none port = conf->sub_topic_name_len
#pragma HLS array_reshape variable = conf->sub_topic_type_name type            \
    = complete                                                 dim = 0
#pragma HLS interface mode = ap_none port = conf->sub_topic_type_name
#pragma HLS interface mode = ap_none port = conf->sub_topic_type_name_len
#pragma HLS interface mode = ap_none port = conf->ignore_ip_checksum
#pragma HLS interface mode = ap_fifo port = pub_app_data
#pragma HLS array_reshape variable = pub_app_data type = complete dim = 0
#pragma HLS interface mode = ap_fifo port = pub_app_data_len
#pragma HLS interface mode = ap_memory port = sub_app_data
#pragma HLS interface mode = ap_none port = sub_app_data_len
#pragma HLS interface mode = ap_none port = sub_app_data_rep_id
#pragma HLS interface mode = ap_vld port = pub_app_data_req
#pragma HLS interface mode = ap_vld port = pub_app_data_rel
#pragma HLS interface mode = ap_none port = pub_app_data_grant
#pragma HLS interface mode = ap_vld port = sub_app_data_recv
#pragma HLS interface mode = ap_vld port = sub_app_data_req
#pragma HLS interface mode = ap_vld port = sub_app_data_rel
#pragma HLS interface mode = ap_none port = sub_app_data_grant
#pragma HLS interface mode = ap_vld port = udp_rxbuf_rel
#pragma HLS interface mode = ap_none port = udp_rxbuf_grant
#pragma HLS interface mode = ap_vld port = udp_txbuf_rel
#pragma HLS interface mode = ap_none port = udp_txbuf_grant
#pragma HLS interface mode = ap_ctrl_none port = return

#pragma HLS interface mode = ap_none port = cnt_interval_elapsed
#pragma HLS interface mode = ap_none port = cnt_spdp_wr_elapsed
#pragma HLS interface mode = ap_none port = cnt_sedp_pub_wr_elapsed
#pragma HLS interface mode = ap_none port = cnt_sedp_sub_wr_elapsed
#pragma HLS interface mode = ap_none port = cnt_sedp_pub_hb_elapsed
#pragma HLS interface mode = ap_none port = cnt_sedp_sub_hb_elapsed
#pragma HLS interface mode = ap_none port = cnt_sedp_pub_an_elapsed
#pragma HLS interface mode = ap_none port = cnt_sedp_sub_an_elapsed
#pragma HLS interface mode = ap_none port = cnt_app_wr_elapsed

#pragma HLS interface mode = ap_vld port = cnt_interval_set
#pragma HLS interface mode = ap_vld port = cnt_spdp_wr_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_pub_wr_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_sub_wr_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_pub_hb_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_sub_hb_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_pub_an_set
#pragma HLS interface mode = ap_vld port = cnt_sedp_sub_an_set
#pragma HLS interface mode = ap_vld port = cnt_app_wr_set

    static sedp_reader_id_t sedp_reader_cnt;
    static app_reader_id_t  app_reader_cnt;

    static sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
    static app_endpoint  app_reader_tbl[APP_READER_MAX];
#pragma HLS array_partition variable = sedp_reader_tbl complete dim = 0
#pragma HLS array_partition variable = app_reader_tbl complete dim = 0

    ros2_in(in, udp_rxbuf, ip_payloads, sedp_reader_cnt, sedp_reader_tbl,
            app_reader_cnt, app_reader_tbl, pub_enable, sub_enable, conf,
            sub_app_data_recv, sub_app_data_req, sub_app_data_rel,
            sub_app_data_grant, sub_app_data, sub_app_data_len,
            sub_app_data_rep_id, udp_rxbuf_rel, udp_rxbuf_grant,
            conf->ignore_ip_checksum, xout);

    ros2_out(
        out, udp_txbuf, sedp_reader_cnt, sedp_reader_tbl, app_reader_cnt,
        app_reader_tbl, pub_enable, sub_enable, conf, pub_app_data,
        pub_app_data_len, pub_app_data_req, pub_app_data_rel,
        pub_app_data_grant, udp_txbuf_rel, udp_txbuf_grant,
        cnt_interval_elapsed, cnt_interval_set, cnt_spdp_wr_elapsed,
        cnt_spdp_wr_set, cnt_sedp_pub_wr_elapsed, cnt_sedp_pub_wr_set,
        cnt_sedp_sub_wr_elapsed, cnt_sedp_sub_wr_set, cnt_sedp_pub_hb_elapsed,
        cnt_sedp_pub_hb_set, cnt_sedp_sub_hb_elapsed, cnt_sedp_sub_hb_set,
        cnt_sedp_pub_an_elapsed, cnt_sedp_pub_an_set, cnt_sedp_sub_an_elapsed,
        cnt_sedp_sub_an_set, cnt_app_wr_elapsed, cnt_app_wr_set);
}
