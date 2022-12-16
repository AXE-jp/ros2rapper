/*
    Copyright © 2021-2022 AXE, Inc. All Rights Reserved.

    This file is part of ROS2rapper.

    ROS2rapper is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ROS2rapper is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with ROS2rapper.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "common.hpp"

#include "app.hpp"
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
	uint8_t buf[TX_BUF_LEN]/* Cyber array=REG */;

	/* Cyber func=inline */
	uint8_t deque()
	{
#pragma HLS inline
#pragma HLS array_partition variable=buf complete dim=0
		return buf[head++];
	}

	/* Cyber func=inline */
	bool empty() const
	{
#pragma HLS inline
		return head == len;
	}

#ifdef USE_FIFOIF_ETHERNET
	/* Cyber func=inline */
	void shift_out(hls_stream<uint8_t> &out)
	{
#pragma HLS inline
		if (!empty() && !out.full())
			out.write(deque());
	}
#else // !USE_FIFOIF_ETHERNET
	/* Cyber func=inline */
	void shift_out(hls_stream<hls_uint<9>> &out)
	{
#pragma HLS inline
		if (!empty() && !out.full()) {
			uint8_t data = deque();
			bool end = empty();

			out.write(data | (end ? 0x100 : 0));
		}
	}
#endif // USE_FIFOIF_ETHERNET
};

#ifdef USE_FIFOIF_ETHERNET
/* Cyber func=inline */
void pre_ip_in(hls_stream<uint8_t> &in, hls_stream<hls_uint<9>> &out)
{
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
static void ros2_in(hls_stream<uint8_t> &in,
		    uint32_t rawudp_rxbuf[],
		    sedp_reader_id_t &sedp_reader_cnt,
		    sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
		    app_reader_id_t &app_reader_cnt,
		    app_endpoint app_reader_tbl[APP_READER_MAX],
		    const config_t &conf,
		    volatile uint8_t *rawudp_rxbuf_rel,
		    volatile uint8_t *rawudp_rxbuf_grant)
{
	static bool ip_parity_error = false;
	static bool udp_parity_error = false;

#pragma HLS inline
	static hls_stream<hls_uint<9>> s1/* Cyber fifo_size=2 */;
	static hls_stream<hls_uint<9>> s2/* Cyber fifo_size=2 */;
	static hls_stream<hls_uint<9>> s3/* Cyber fifo_size=2 */;
	static hls_stream<hls_uint<9>> s4/* Cyber fifo_size=2 */;
	static hls_stream<hls_uint<9>> s5/* Cyber fifo_size=2 */;
#pragma HLS stream variable=s1 depth=2
#pragma HLS stream variable=s2 depth=2
#pragma HLS stream variable=s3 depth=2
#pragma HLS stream variable=s4 depth=2
#pragma HLS stream variable=s5 depth=2

	hls_uint<9> x;

#ifdef USE_FIFOIF_ETHERNET
	pre_ip_in(in, s1);
#else // !USE_FIFOIF_ETHERNET
	slip_in(in, s1);
#endif // USE_FIFOIF_ETHERNET
	ip_in(s1, s2, ip_parity_error);
	udp_in(s2, s3, conf.cpu_udp_port, rawudp_rxbuf, rawudp_rxbuf_rel, rawudp_rxbuf_grant, udp_parity_error);

	if (!s3.read_nb(x))
		return;

	s4.write(x);
	s5.write(x);

	spdp_reader(s4,
		    sedp_reader_cnt,
		    sedp_reader_tbl,
		    conf.port_num_seed);
	sedp_reader(s5,
		    app_reader_cnt,
		    app_reader_tbl,
		    conf.port_num_seed,
		    conf.guid_prefix,
		    conf.topic_name,
		    conf.topic_name_len,
		    conf.topic_type_name,
		    conf.topic_type_name_len);
}

/* Cyber func=process */
static void spdp_writer_out(const uint8_t metatraffic_port[2],
			    const uint8_t default_port[2],
			    tx_buf &tx_buf,
			    const config_t &conf)
{
	static const uint8_t dst_addr[4]/* Cyber array=REG */ =
		IP_MULTICAST_ADDR;
	uint8_t dst_port[2]/* Cyber array=REG */;
	dst_port[0] = DISCOVERY_TRAFFIC_MULTICAST_PORT_0(conf.port_num_seed);
	dst_port[1] = DISCOVERY_TRAFFIC_MULTICAST_PORT_1(conf.port_num_seed);
#pragma HLS array_partition variable=dst_addr complete dim=0
#pragma HLS array_partition variable=dst_port complete dim=0

	ip_set_header(conf.ip_addr,
	       dst_addr,
	       IP_HDR_TTL_MULTICAST,
	       SPDP_WRITER_UDP_PKT_LEN,
	       tx_buf.buf);

	udp_set_header(conf.node_udp_port,
		dst_port,
		SPDP_WRITER_RTPS_PKT_LEN,
		tx_buf.buf + IP_HDR_SIZE);

	spdp_writer(conf.guid_prefix,
		    conf.ip_addr,
		    metatraffic_port,
		    conf.ip_addr,
		    default_port,
		    tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE),
		    conf.node_name,
		    conf.node_name_len);

	tx_buf.head = 0;
	tx_buf.len = SPDP_WRITER_IP_PKT_LEN;
}

/* Cyber func=process */
static void sedp_writer_out(const uint8_t writer_entity_id[4],
			    const uint8_t dst_addr[4],
			    const uint8_t dst_port[2],
			    const uint8_t reader_guid_prefix[12],
			    const uint8_t reader_entity_id[4],
			    const uint8_t usertraffic_port[2],
			    const uint8_t app_entity_id[4],
			    tx_buf &tx_buf,
			    const config_t &conf)
{
	ip_set_header(conf.ip_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
	       SEDP_WRITER_UDP_PKT_LEN,
	       tx_buf.buf);

	udp_set_header(conf.node_udp_port,
		dst_port,
		SEDP_WRITER_RTPS_PKT_LEN,
		tx_buf.buf + IP_HDR_SIZE);

	sedp_writer(conf.guid_prefix,
		    writer_entity_id,
		    reader_guid_prefix,
		    reader_entity_id,
		    conf.ip_addr,
		    usertraffic_port,
		    app_entity_id,
		    tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE),
		    conf.topic_name,
		    conf.topic_name_len,
		    conf.topic_type_name,
		    conf.topic_type_name_len);

	tx_buf.head = 0;
	tx_buf.len = SEDP_WRITER_IP_PKT_LEN;
}

/* Cyber func=process */
static void sedp_heartbeat_out(const uint8_t writer_entity_id[4],
			       const uint8_t dst_addr[4],
			       const uint8_t dst_port[2],
			       const uint8_t reader_guid_prefix[12],
			       const uint8_t reader_entity_id[4],
			       const int64_t first_seqnum,
			       const int64_t last_seqnum,
			       tx_buf &tx_buf,
			       uint32_t &cnt,
			       const uint8_t src_addr[4],
			       const uint8_t src_port[2],
			       const uint8_t writer_guid_prefix[12])
{
	cnt++;

	ip_set_header(src_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
	       SEDP_HEARTBEAT_UDP_PKT_LEN,
	       tx_buf.buf);

	udp_set_header(src_port,
		dst_port,
		SEDP_HEARTBEAT_RTPS_PKT_LEN,
		tx_buf.buf + IP_HDR_SIZE);

	sedp_heartbeat(writer_guid_prefix,
		       writer_entity_id,
		       reader_guid_prefix,
		       reader_entity_id,
		       first_seqnum,
		       last_seqnum,
		       cnt,
		       tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE));

	tx_buf.head = 0;
	tx_buf.len = SEDP_HEARTBEAT_IP_PKT_LEN;
}

/* Cyber func=process */
static void sedp_acknack_out(const uint8_t writer_entity_id[4],
			     const uint8_t dst_addr[4],
			     const uint8_t dst_port[2],
			     const uint8_t reader_guid_prefix[12],
			     const uint8_t reader_entity_id[4],
			     const int64_t bitmap_base,
			     const uint32_t num_bits,
			     const uint8_t bitmap[4],
			     tx_buf &tx_buf,
			     uint32_t &cnt,
			     const uint8_t src_addr[4],
			     const uint8_t src_port[2],
			     const uint8_t writer_guid_prefix[12])
{
	cnt++;

	ip_set_header(src_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
	       SEDP_ACKNACK_UDP_PKT_LEN,
	       tx_buf.buf);

	udp_set_header(src_port,
		dst_port,
		SEDP_ACKNACK_RTPS_PKT_LEN,
		tx_buf.buf + IP_HDR_SIZE);

	sedp_acknack(writer_guid_prefix,
		     writer_entity_id,
		     reader_guid_prefix,
		     reader_entity_id,
		     bitmap_base,
		     num_bits,
		     bitmap,
		     cnt,
		     tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE));

	tx_buf.head = 0;
	tx_buf.len = SEDP_ACKNACK_IP_PKT_LEN;
}

/* Cyber func=process */
static void app_writer_out(const uint8_t writer_entity_id[4],
			   const uint8_t dst_addr[4],
			   const uint8_t dst_port[2],
			   const uint8_t reader_guid_prefix[12],
			   const uint8_t reader_entity_id[4],
			   tx_buf &tx_buf,
			   int64_t &seqnum,
			   const uint8_t src_addr[4],
			   const uint8_t src_port[2],
			   const uint8_t writer_guid_prefix[12],
			   const uint8_t app_data[MAX_APP_DATA_LEN],
			   uint8_t app_data_len)
{
	seqnum++;

	ip_set_header(src_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
	       APP_WRITER_UDP_PKT_LEN,
	       tx_buf.buf);

	udp_set_header(src_port,
		dst_port,
		APP_WRITER_RTPS_PKT_LEN,
		tx_buf.buf + IP_HDR_SIZE);

	app_writer(writer_guid_prefix,
		   writer_entity_id,
		   reader_guid_prefix,
		   reader_entity_id,
		   seqnum,
		   app_data,
		   app_data_len,
		   tx_buf.buf + (IP_HDR_SIZE + UDP_HDR_SIZE));

	tx_buf.head = 0;
	tx_buf.len = APP_WRITER_IP_PKT_LEN;
}

/* Cyber func=process */
static void rawudp_out(const uint8_t dst_addr[4],
			   const uint8_t dst_port[2],
			   tx_buf &tx_buf,
			   const uint8_t src_addr[4],
			   const uint8_t src_port[2],
			   uint8_t udp_payload_len)
{
	ip_set_header(src_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
				 udp_payload_len + UDP_HDR_SIZE,
	       tx_buf.buf);

	udp_set_header(src_port,
		dst_port,
		udp_payload_len,
		tx_buf.buf + IP_HDR_SIZE);

	tx_buf.head = 0;
	tx_buf.len = IP_HDR_SIZE + UDP_HDR_SIZE + udp_payload_len;
}

#define ST_RAWUDP_OUT           0
#define ST_SPDP_WRITER           1
#define ST_SEDP_PUB_WRITER_0     2
#define ST_SEDP_PUB_WRITER_1     3
#define ST_SEDP_PUB_WRITER_2     4
#define ST_SEDP_PUB_WRITER_3     5
#define ST_SEDP_SUB_WRITER_0     6
#define ST_SEDP_SUB_WRITER_1     7
#define ST_SEDP_SUB_WRITER_2     8
#define ST_SEDP_SUB_WRITER_3     9
#define ST_SEDP_PUB_HEARTBEAT_0  10
#define ST_SEDP_PUB_HEARTBEAT_1  11
#define ST_SEDP_PUB_HEARTBEAT_2  12
#define ST_SEDP_PUB_HEARTBEAT_3  13
#define ST_SEDP_SUB_HEARTBEAT_0  14
#define ST_SEDP_SUB_HEARTBEAT_1  15
#define ST_SEDP_SUB_HEARTBEAT_2  16
#define ST_SEDP_SUB_HEARTBEAT_3  17
#define ST_SEDP_PUB_ACKNACK_0    18
#define ST_SEDP_PUB_ACKNACK_1    19
#define ST_SEDP_PUB_ACKNACK_2    20
#define ST_SEDP_PUB_ACKNACK_3    21
#define ST_SEDP_SUB_ACKNACK_0    22
#define ST_SEDP_SUB_ACKNACK_1    23
#define ST_SEDP_SUB_ACKNACK_2    24
#define ST_SEDP_SUB_ACKNACK_3    25
#define ST_APP_WRITER_0          26
#define ST_APP_WRITER_1          27
#define ST_APP_WRITER_2          28
#define ST_APP_WRITER_3          29

#define SPDP_WRITER_OUT()						\
	do {								\
		spdp_writer_out(metatraffic_port,			\
				default_port,				\
				tx_buf,				\
				conf);				\
	} while (0)

#define SEDP_PUB_WRITER_OUT(id)						\
	do {								\
		if (sedp_reader_cnt > (id)) {				\
			sedp_writer_out(				\
				pub_writer_entity_id,			\
				sedp_reader_tbl[(id)].ip_addr,		\
				sedp_reader_tbl[(id)].udp_port,		\
				sedp_reader_tbl[(id)].guid_prefix,	\
				pub_reader_entity_id,			\
				default_port,				\
				app_writer_entity_id,			\
				tx_buf,				\
				conf);				\
		}							\
	} while (0)

#define SEDP_SUB_WRITER_OUT(id)	do {} while (0) // do nothing

#define SEDP_PUB_HEARTBEAT_OUT(id)					\
	do {								\
		if (sedp_reader_cnt > (id)) {				\
			sedp_heartbeat_out(				\
				pub_writer_entity_id,			\
				sedp_reader_tbl[(id)].ip_addr,		\
				sedp_reader_tbl[(id)].udp_port,		\
				sedp_reader_tbl[(id)].guid_prefix,	\
				pub_reader_entity_id,			\
				1,					\
				1,					\
				tx_buf,					\
				sedp_pub_heartbeat_cnt[(id)],		\
				conf.ip_addr,		\
				conf.node_udp_port,		\
				conf.guid_prefix);		\
		}							\
	} while (0)

#define SEDP_SUB_HEARTBEAT_OUT(id)					\
	do {								\
		if (sedp_reader_cnt > (id)) {				\
			sedp_heartbeat_out(				\
				sub_writer_entity_id,			\
				sedp_reader_tbl[(id)].ip_addr,		\
				sedp_reader_tbl[(id)].udp_port,		\
				sedp_reader_tbl[(id)].guid_prefix,	\
				sub_reader_entity_id,			\
				1,					\
				0,					\
				tx_buf,					\
				sedp_sub_heartbeat_cnt[(id)],		\
				conf.ip_addr,		\
				conf.node_udp_port,		\
				conf.guid_prefix);		\
		}							\
	} while (0)

#define SEDP_PUB_ACKNACK_OUT(id)					\
	do {								\
		if (sedp_reader_cnt > (id)) {				\
			sedp_acknack_out(				\
				pub_writer_entity_id,			\
				sedp_reader_tbl[(id)].ip_addr,		\
				sedp_reader_tbl[(id)].udp_port,		\
				sedp_reader_tbl[(id)].guid_prefix,	\
				pub_reader_entity_id,			\
				1,					\
				32,					\
				bitmap,					\
				tx_buf,					\
				sedp_pub_acknack_cnt[(id)],		\
				conf.ip_addr,		\
				conf.node_udp_port,		\
				conf.guid_prefix);		\
		}							\
	} while (0)

#define SEDP_SUB_ACKNACK_OUT(id)					\
	do {								\
		if (sedp_reader_cnt > (id)) {				\
			sedp_acknack_out(				\
				sub_writer_entity_id,			\
				sedp_reader_tbl[(id)].ip_addr,		\
				sedp_reader_tbl[(id)].udp_port,		\
				sedp_reader_tbl[(id)].guid_prefix,	\
				sub_reader_entity_id,			\
				1,					\
				32,					\
				bitmap,					\
				tx_buf,					\
				sedp_sub_acknack_cnt[(id)],		\
				conf.ip_addr,		\
				conf.node_udp_port,		\
				conf.guid_prefix);		\
		}							\
	} while (0)

#define REQ_APP_DATA \
	do { \
		*app_data_req = 0/*write dummy value to assert ap_vld*/; \
		ap_wait(); \
		ap_wait(); \
		grant = *app_data_grant; \
	} while (0)

#define REL_APP_DATA \
	do { \
		*app_data_rel = 0/*write dummy value to assert ap_vld*/; \
		ap_wait(); \
	} while (0)

#define APP_WRITER_OUT(id)						\
	do {								\
		if (app_reader_cnt > (id)) {				\
			REQ_APP_DATA; \
			if (grant == 1) { \
				app_writer_out(					\
					app_writer_entity_id,			\
					app_reader_tbl[(id)].ip_addr,		\
					app_reader_tbl[(id)].udp_port,		\
					app_reader_tbl[(id)].guid_prefix,	\
					app_reader_tbl[(id)].entity_id,		\
					tx_buf,					\
					app_seqnum,				\
					conf.ip_addr,				\
					conf.node_udp_port,				\
					conf.guid_prefix,				\
					conf.app_data,				\
					conf.app_data_len);				\
			} \
			REL_APP_DATA; \
		}							\
	} while (0)

#define RAWUDP_OUT()					\
	do {								\
		rawudp_out(				\
			rawudp_tx_dst_addr,					\
			rawudp_tx_dst_port,					\
			tx_buf,					\
			conf.ip_addr,		\
			rawudp_tx_src_port,					\
			rawudp_tx_payload_len);		\
	} while (0)

/* Cyber func=inline */
static void ros2_out(hls_stream<uint8_t> &out,
		     uint32_t rawudp_txbuf[],
		     sedp_reader_id_t &sedp_reader_cnt,
		     sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
		     app_reader_id_t &app_reader_cnt,
		     app_endpoint app_reader_tbl[APP_READER_MAX],
		     const config_t &conf,
		     volatile uint8_t *app_data_req,
		     volatile uint8_t *app_data_rel,
		     volatile uint8_t *app_data_grant,
		     volatile uint8_t *rawudp_txbuf_rel,
		     volatile uint8_t *rawudp_txbuf_grant)
{
#pragma HLS inline
	static const uint8_t pub_writer_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_BUILTIN_PUBLICATIONS_WRITER;
	static const uint8_t sub_writer_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_BUILTIN_SUBSCRIPTIONS_WRITER;
	static const uint8_t app_writer_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_APP_WRITER;
#pragma HLS array_partition variable=pub_writer_entity_id complete dim=0
#pragma HLS array_partition variable=sub_writer_entity_id complete dim=0
#pragma HLS array_partition variable=app_writer_entity_id complete dim=0

	static const uint8_t pub_reader_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_BUILTIN_PUBLICATIONS_READER;
	static const uint8_t sub_reader_entity_id[4]/* Cyber array=REG */ =
		ENTITYID_BUILTIN_SUBSCRIPTIONS_READER;
#pragma HLS array_partition variable=pub_reader_entity_id complete dim=0
#pragma HLS array_partition variable=sub_reader_entity_id complete dim=0

	uint8_t metatraffic_port[2]/* Cyber array=REG */;
	metatraffic_port[0] = DISCOVERY_TRAFFIC_UNICAST_PORT_0(conf.port_num_seed, TARGET_PARTICIPANT_ID);
	metatraffic_port[1] = DISCOVERY_TRAFFIC_UNICAST_PORT_1(conf.port_num_seed, TARGET_PARTICIPANT_ID);

	uint8_t default_port[2]/* Cyber array=REG */;
	default_port[0] = USER_TRAFFIC_UNICAST_PORT_0(conf.port_num_seed, TARGET_PARTICIPANT_ID);
	default_port[1] = USER_TRAFFIC_UNICAST_PORT_1(conf.port_num_seed, TARGET_PARTICIPANT_ID);

#pragma HLS array_partition variable=metatraffic_port complete dim=0
#pragma HLS array_partition variable=default_port complete dim=0

	static const uint8_t bitmap[4]/* Cyber array=REG */ =
		{0xff, 0xff, 0xff, 0xff};
#pragma HLS array_partition variable=bitmap complete dim=0

	static tx_buf tx_buf;

	static uint32_t sedp_pub_heartbeat_cnt[SEDP_READER_MAX];
	static uint32_t sedp_sub_heartbeat_cnt[SEDP_READER_MAX];
	static uint32_t sedp_pub_acknack_cnt[SEDP_READER_MAX];
	static uint32_t sedp_sub_acknack_cnt[SEDP_READER_MAX];
#pragma HLS array_partition variable=sedp_pub_heartbeat_cnt complete dim=0
#pragma HLS array_partition variable=sedp_sub_heartbeat_cnt complete dim=0
#pragma HLS array_partition variable=sedp_pub_acknack_cnt complete dim=0
#pragma HLS array_partition variable=sedp_sub_acknack_cnt complete dim=0

	static int64_t app_seqnum;

#ifndef USE_FIFOIF_ETHERNET
	static hls_stream<hls_uint<9>> s/* Cyber fifo_size=2 */;
#pragma HLS stream variable=s depth=2
#endif // !USE_FIFOIF_ETHERNET

	static uint8_t rawudp_tx_dst_addr[4]/* Cyber array=REG */;
	static uint8_t rawudp_tx_dst_port[2]/* Cyber array=REG */;
	static uint8_t rawudp_tx_src_port[2]/* Cyber array=REG */;
#pragma HLS array_partition variable=rawudp_tx_dst_addr complete dim=0
#pragma HLS array_partition variable=rawudp_tx_dst_port complete dim=0
#pragma HLS array_partition variable=rawudp_tx_src_port complete dim=0
	static uint16_t rawudp_tx_payload_len;
	uint32_t ram_read_buf;

#define RAWUDP_TXBUF_COPY_INIT    0
#define RAWUDP_TXBUF_COPY_RUNNING 1
#define RAWUDP_TXBUF_COPY_DONE    2
	static hls_uint<2> rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_INIT;
	static uint16_t rawudp_txbuf_rd_off;
	static uint16_t rawudp_txpayload_wr_off;

	static uint32_t clk_cnt;
	static hls_uint<5> state = ST_RAWUDP_OUT;

	uint8_t grant;

	clk_cnt++;

	if (!tx_buf.empty()) {
#ifdef USE_FIFOIF_ETHERNET
		tx_buf.shift_out(out);
#else // !USE_FIFOIF_ETHERNET
		tx_buf.shift_out(s);

		slip_out(s, out);
#endif // USE_FIFOIF_ETHERNET
	} else {
		switch (state) {
		case ST_RAWUDP_OUT:
			switch (rawudp_txbuf_copy_status) {
			case RAWUDP_TXBUF_COPY_INIT:
				if (*rawudp_txbuf_grant == 1) {
					rawudp_txbuf_rd_off = 0;
					rawudp_txpayload_wr_off = 0;
					rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_RUNNING;
				} else if (clk_cnt >= conf.tx_period) {
					clk_cnt = 0;
					state++;
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
						tx_buf.buf[rawudp_txpayload_wr_off + 0 + IP_HDR_SIZE + UDP_HDR_SIZE] = (rawudp_tx_payload_len <= rawudp_txpayload_wr_off + 0) ? 0 : (ram_read_buf & 0xff);
						tx_buf.buf[rawudp_txpayload_wr_off + 1 + IP_HDR_SIZE + UDP_HDR_SIZE] = (rawudp_tx_payload_len <= rawudp_txpayload_wr_off + 1) ? 0 : ((ram_read_buf >> 8) & 0xff);
						tx_buf.buf[rawudp_txpayload_wr_off + 2 + IP_HDR_SIZE + UDP_HDR_SIZE] = (rawudp_tx_payload_len <= rawudp_txpayload_wr_off + 2) ? 0 : ((ram_read_buf >> 16) & 0xff);
						tx_buf.buf[rawudp_txpayload_wr_off + 3 + IP_HDR_SIZE + UDP_HDR_SIZE] = (rawudp_tx_payload_len <= rawudp_txpayload_wr_off + 3) ? 0 : ((ram_read_buf >> 24) & 0xff);
						rawudp_txpayload_wr_off += 4;
					}
					break;
				}
				rawudp_txbuf_rd_off++;
				break;
			case RAWUDP_TXBUF_COPY_DONE:
				RAWUDP_OUT();
				*rawudp_txbuf_rel = 0 /*write dummy value to assert ap_vld*/; \
				rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_INIT;
				if (clk_cnt >= conf.tx_period) {
					clk_cnt = 0;
					state++;
				}
				break;
			default:
				rawudp_txbuf_copy_status = RAWUDP_TXBUF_COPY_INIT;
				break;
			}
			break;
		case ST_SPDP_WRITER: SPDP_WRITER_OUT(); state++; break;
		case ST_SEDP_PUB_WRITER_0:    SEDP_PUB_WRITER_OUT(0); state++; break;
		case ST_SEDP_PUB_WRITER_1:    SEDP_PUB_WRITER_OUT(1); state++; break;
		case ST_SEDP_PUB_WRITER_2:    SEDP_PUB_WRITER_OUT(2); state++; break;
		case ST_SEDP_PUB_WRITER_3:    SEDP_PUB_WRITER_OUT(3); state++; break;
		case ST_SEDP_SUB_WRITER_0:    SEDP_SUB_WRITER_OUT(0); state++; break;
		case ST_SEDP_SUB_WRITER_1:    SEDP_SUB_WRITER_OUT(1); state++; break;
		case ST_SEDP_SUB_WRITER_2:    SEDP_SUB_WRITER_OUT(2); state++; break;
		case ST_SEDP_SUB_WRITER_3:    SEDP_SUB_WRITER_OUT(3); state++; break;
		case ST_SEDP_PUB_HEARTBEAT_0: SEDP_PUB_HEARTBEAT_OUT(0); state++; break;
		case ST_SEDP_PUB_HEARTBEAT_1: SEDP_PUB_HEARTBEAT_OUT(1); state++; break;
		case ST_SEDP_PUB_HEARTBEAT_2: SEDP_PUB_HEARTBEAT_OUT(2); state++; break;
		case ST_SEDP_PUB_HEARTBEAT_3: SEDP_PUB_HEARTBEAT_OUT(3); state++; break;
		case ST_SEDP_SUB_HEARTBEAT_0: SEDP_SUB_HEARTBEAT_OUT(0); state++; break;
		case ST_SEDP_SUB_HEARTBEAT_1: SEDP_SUB_HEARTBEAT_OUT(1); state++; break;
		case ST_SEDP_SUB_HEARTBEAT_2: SEDP_SUB_HEARTBEAT_OUT(2); state++; break;
		case ST_SEDP_SUB_HEARTBEAT_3: SEDP_SUB_HEARTBEAT_OUT(3); state++; break;
		case ST_SEDP_PUB_ACKNACK_0:   SEDP_PUB_ACKNACK_OUT(0); state++; break;
		case ST_SEDP_PUB_ACKNACK_1:   SEDP_PUB_ACKNACK_OUT(1); state++; break;
		case ST_SEDP_PUB_ACKNACK_2:   SEDP_PUB_ACKNACK_OUT(2); state++; break;
		case ST_SEDP_PUB_ACKNACK_3:   SEDP_PUB_ACKNACK_OUT(3); state++; break;
		case ST_SEDP_SUB_ACKNACK_0:   SEDP_SUB_ACKNACK_OUT(0); state++; break;
		case ST_SEDP_SUB_ACKNACK_1:   SEDP_SUB_ACKNACK_OUT(1); state++; break;
		case ST_SEDP_SUB_ACKNACK_2:   SEDP_SUB_ACKNACK_OUT(2); state++; break;
		case ST_SEDP_SUB_ACKNACK_3:   SEDP_SUB_ACKNACK_OUT(3); state++; break;
		case ST_APP_WRITER_0:         APP_WRITER_OUT(0); state++; break;
		case ST_APP_WRITER_1:         APP_WRITER_OUT(1); state++; break;
		case ST_APP_WRITER_2:         APP_WRITER_OUT(2); state++; break;
		case ST_APP_WRITER_3:         APP_WRITER_OUT(3);
		default:                      state = ST_RAWUDP_OUT; break;
		}

		if (!tx_buf.empty()) {
			ip_set_checksum(tx_buf.buf);
			udp_set_checksum(tx_buf.buf);
		}
	}
}

/* Cyber func=process_pipeline */
void ros2(hls_stream<uint8_t> &in/* Cyber port_mode=cw_fifo */,
	  hls_stream<uint8_t> &out/* Cyber port_mode=cw_fifo */,
	  uint32_t udp_rxbuf[RAWUDP_RXBUF_LEN/4],
	  uint32_t udp_txbuf[RAWUDP_TXBUF_LEN/4],
	  const config_t &conf,
	  volatile uint8_t *app_data_req,
	  volatile uint8_t *app_data_rel,
	  volatile uint8_t *app_data_grant,
	  volatile uint8_t *udp_rxbuf_rel,
	  volatile uint8_t *udp_rxbuf_grant,
	  volatile uint8_t *udp_txbuf_rel,
	  volatile uint8_t *udp_txbuf_grant)
{
#pragma HLS interface ap_fifo port=in
#pragma HLS interface ap_fifo port=out
#pragma HLS interface ap_memory port=udp_rxbuf
#pragma HLS interface ap_memory port=udp_txbuf storage_type=ram_1p
#pragma HLS interface ap_none port=conf
#pragma HLS disaggregate variable=conf
#pragma HLS interface ap_vld port=app_data_req
#pragma HLS interface ap_vld port=app_data_rel
#pragma HLS interface ap_none port=app_data_grant
#pragma HLS interface ap_vld port=udp_rxbuf_rel
#pragma HLS interface ap_none port=udp_rxbuf_grant
#pragma HLS interface ap_vld port=udp_txbuf_rel
#pragma HLS interface ap_none port=udp_txbuf_grant
#pragma HLS interface ap_ctrl_none port=return

	static sedp_reader_id_t sedp_reader_cnt;
	static app_reader_id_t app_reader_cnt;

	static sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
	static app_endpoint app_reader_tbl[APP_READER_MAX];
#pragma HLS array_partition variable=sedp_reader_tbl complete dim=0
#pragma HLS array_partition variable=app_reader_tbl complete dim=0

	ros2_in(in,
		udp_rxbuf,
		sedp_reader_cnt,
		sedp_reader_tbl,
		app_reader_cnt,
		app_reader_tbl,
		conf,
		udp_rxbuf_rel,
		udp_rxbuf_grant);

	ros2_out(out,
		 udp_txbuf,
		 sedp_reader_cnt,
		 sedp_reader_tbl,
		 app_reader_cnt,
		 app_reader_tbl,
		 conf,
		 app_data_req,
		 app_data_rel,
		 app_data_grant,
		 udp_txbuf_rel,
		 udp_txbuf_grant);
}
