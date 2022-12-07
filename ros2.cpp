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


#define SPDP_WRITER_RTPS_PKT_LEN			\
	SPDP_WRITER_TOT_LEN
#define SPDP_WRITER_UDP_PKT_LEN				\
	(UDP_HDR_SIZE + SPDP_WRITER_RTPS_PKT_LEN)
#define SPDP_WRITER_IP_PKT_LEN				\
	(IP_HDR_SIZE + SPDP_WRITER_UDP_PKT_LEN)

#define SEDP_WRITER_RTPS_PKT_LEN			\
	SEDP_WRITER_TOT_LEN
#define SEDP_WRITER_UDP_PKT_LEN				\
	(UDP_HDR_SIZE + SEDP_WRITER_RTPS_PKT_LEN)
#define SEDP_WRITER_IP_PKT_LEN				\
	(IP_HDR_SIZE + SEDP_WRITER_UDP_PKT_LEN)

#define SEDP_HEARTBEAT_RTPS_PKT_LEN			\
	SEDP_HEARTBEAT_TOT_LEN
#define SEDP_HEARTBEAT_UDP_PKT_LEN			\
	(UDP_HDR_SIZE + SEDP_HEARTBEAT_RTPS_PKT_LEN)
#define SEDP_HEARTBEAT_IP_PKT_LEN			\
	(IP_HDR_SIZE + SEDP_HEARTBEAT_UDP_PKT_LEN)

#define SEDP_ACKNACK_RTPS_PKT_LEN			\
	SEDP_ACKNACK_TOT_LEN
#define SEDP_ACKNACK_UDP_PKT_LEN			\
	(UDP_HDR_SIZE + SEDP_ACKNACK_RTPS_PKT_LEN)
#define SEDP_ACKNACK_IP_PKT_LEN				\
	(IP_HDR_SIZE + SEDP_ACKNACK_UDP_PKT_LEN)

#define APP_WRITER_RTPS_PKT_LEN				\
	APP_TOT_LEN(MAX_APP_DATA_LEN)
#define APP_WRITER_UDP_PKT_LEN				\
	(UDP_HDR_SIZE + APP_WRITER_RTPS_PKT_LEN)
#define APP_WRITER_IP_PKT_LEN				\
	(IP_HDR_SIZE + APP_WRITER_UDP_PKT_LEN)


#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define TX_BUF_SIZE	(MAX(MAX(MAX(MAX(SPDP_WRITER_IP_PKT_LEN, SEDP_WRITER_IP_PKT_LEN), SEDP_HEARTBEAT_IP_PKT_LEN), SEDP_ACKNACK_IP_PKT_LEN), APP_WRITER_IP_PKT_LEN))

class tx_buf {
public:
	uint16_t head;
	uint16_t len;
	uint8_t buf[TX_BUF_SIZE]/* Cyber array=REG */;

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
		if (!empty())
			out.write(deque());
	}
#else // !USE_FIFOIF_ETHERNET
	/* Cyber func=inline */
	void shift_out(hls_stream<hls_uint<9>> &out)
	{
#pragma HLS inline
		if (!empty()) {
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
		    sedp_reader_id_t &sedp_reader_cnt,
		    sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
		    app_reader_id_t &app_reader_cnt,
		    app_endpoint app_reader_tbl[APP_READER_MAX],
		    const config_t &conf)
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
	udp_in(s2, s3, udp_parity_error);

	if (!s3.read_nb(x))
		return;

	if (conf.ctrl & CTRL_ENABLE) {
		s4.write(x);
		s5.write(x);
	}

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

	uint8_t rtps_pkt[SPDP_WRITER_RTPS_PKT_LEN]/* Cyber array=REG */;
	uint8_t udp_pkt[SPDP_WRITER_UDP_PKT_LEN]/* Cyber array=REG */;
#pragma HLS array_partition variable=rtps_pkt complete dim=0
#pragma HLS array_partition variable=udp_pkt complete dim=0

	spdp_writer(conf.guid_prefix,
		    conf.ip_addr,
		    metatraffic_port,
		    conf.ip_addr,
		    default_port,
		    rtps_pkt,
		    conf.node_name,
		    conf.node_name_len);

	udp_out(conf.ip_addr,
		conf.node_udp_port,
		dst_addr,
		dst_port,
		rtps_pkt,
		SPDP_WRITER_RTPS_PKT_LEN,
		udp_pkt);

	ip_out(conf.ip_addr,
	       dst_addr,
	       IP_HDR_TTL_MULTICAST,
	       udp_pkt,
	       SPDP_WRITER_UDP_PKT_LEN,
	       tx_buf.buf);

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
	uint8_t rtps_pkt[SEDP_WRITER_RTPS_PKT_LEN]/* Cyber array=REG */;
	uint8_t udp_pkt[SEDP_WRITER_UDP_PKT_LEN]/* Cyber array=REG */;
#pragma HLS array_partition variable=rtps_pkt complete dim=0
#pragma HLS array_partition variable=udp_pkt complete dim=0

	sedp_writer(conf.guid_prefix,
		    writer_entity_id,
		    reader_guid_prefix,
		    reader_entity_id,
		    conf.ip_addr,
		    usertraffic_port,
		    app_entity_id,
		    rtps_pkt,
		    conf.topic_name,
		    conf.topic_name_len,
		    conf.topic_type_name,
		    conf.topic_type_name_len);

	udp_out(conf.ip_addr,
		conf.node_udp_port,
		dst_addr,
		dst_port,
		rtps_pkt,
		SEDP_WRITER_RTPS_PKT_LEN,
		udp_pkt);

	ip_out(conf.ip_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
	       udp_pkt,
	       SEDP_WRITER_UDP_PKT_LEN,
	       tx_buf.buf);

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
	uint8_t rtps_pkt[SEDP_HEARTBEAT_RTPS_PKT_LEN]/* Cyber array=REG */;
	uint8_t udp_pkt[SEDP_HEARTBEAT_UDP_PKT_LEN]/* Cyber array=REG */;
#pragma HLS array_partition variable=rtps_pkt complete dim=0
#pragma HLS array_partition variable=udp_pkt complete dim=0

	cnt++;
	sedp_heartbeat(writer_guid_prefix,
		       writer_entity_id,
		       reader_guid_prefix,
		       reader_entity_id,
		       first_seqnum,
		       last_seqnum,
		       cnt,
		       rtps_pkt);

	udp_out(src_addr,
		src_port,
		dst_addr,
		dst_port,
		rtps_pkt,
		SEDP_HEARTBEAT_RTPS_PKT_LEN,
		udp_pkt);

	ip_out(src_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
	       udp_pkt,
	       SEDP_HEARTBEAT_UDP_PKT_LEN,
	       tx_buf.buf);

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
	uint8_t rtps_pkt[SEDP_ACKNACK_RTPS_PKT_LEN]/* Cyber array=REG */;
	uint8_t udp_pkt[SEDP_ACKNACK_UDP_PKT_LEN]/* Cyber array=REG */;
#pragma HLS array_partition variable=rtps_pkt complete dim=0
#pragma HLS array_partition variable=udp_pkt complete dim=0

	cnt++;
	sedp_acknack(writer_guid_prefix,
		     writer_entity_id,
		     reader_guid_prefix,
		     reader_entity_id,
		     bitmap_base,
		     num_bits,
		     bitmap,
		     cnt,
		     rtps_pkt);

	udp_out(src_addr,
		src_port,
		dst_addr,
		dst_port,
		rtps_pkt,
		SEDP_ACKNACK_RTPS_PKT_LEN,
		udp_pkt);

	ip_out(src_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
	       udp_pkt,
	       SEDP_ACKNACK_UDP_PKT_LEN,
	       tx_buf.buf);

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
	uint8_t rtps_pkt[APP_WRITER_RTPS_PKT_LEN]/* Cyber array=REG */;
	uint8_t udp_pkt[APP_WRITER_UDP_PKT_LEN]/* Cyber array=REG */;
#pragma HLS array_partition variable=rtps_pkt complete dim=0
#pragma HLS array_partition variable=udp_pkt complete dim=0

	seqnum++;
	app_writer(writer_guid_prefix,
		   writer_entity_id,
		   reader_guid_prefix,
		   reader_entity_id,
		   seqnum,
		   app_data,
		   app_data_len,
		   rtps_pkt);

	udp_out(src_addr,
		src_port,
		dst_addr,
		dst_port,
		rtps_pkt,
		APP_WRITER_RTPS_PKT_LEN,
		udp_pkt);

	ip_out(src_addr,
	       dst_addr,
	       IP_HDR_TTL_UNICAST,
	       udp_pkt,
	       APP_WRITER_UDP_PKT_LEN,
	       tx_buf.buf);

	tx_buf.head = 0;
	tx_buf.len = APP_WRITER_IP_PKT_LEN;
}

#define CLOCK_COUNT_CYCLE	(TARGET_CLOCK_FREQ / 8)

#define SPDP_WRITER_CYCLE		0
#define SEDP_PUB_WRITER_CYCLE(id)			\
	((CLOCK_COUNT_CYCLE / 100) * (10 + (id)))
#define SEDP_SUB_WRITER_CYCLE(id)			\
	((CLOCK_COUNT_CYCLE / 100) * (20 + (id)))
#define SEDP_PUB_HEARTBEAT_CYCLE(id)			\
	((CLOCK_COUNT_CYCLE / 100) * (30 + (id)))
#define SEDP_SUB_HEARTBEAT_CYCLE(id)			\
	((CLOCK_COUNT_CYCLE / 100) * (40 + (id)))
#define SEDP_PUB_ACKNACK_CYCLE(id)			\
	((CLOCK_COUNT_CYCLE / 100) * (50 + (id)))
#define SEDP_SUB_ACKNACK_CYCLE(id)			\
	((CLOCK_COUNT_CYCLE / 100) * (60 + (id)))
#define APP_WRITER_CYCLE(id)				\
	((CLOCK_COUNT_CYCLE / 100) * (70 + (id)))

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

#define APP_WRITER_OUT(id)						\
	do {								\
		*app_data_req = 0; ap_wait(); ap_wait(); \
		grant = *app_data_grant; \
		if (grant == 1 && app_reader_cnt > (id)) {				\
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
				*app_data_rel = 0; ap_wait(); ap_wait(); \
		}							\
	} while (0)							\

/* Cyber func=inline */
static void ros2_out(hls_stream<uint8_t> &out,
		     sedp_reader_id_t &sedp_reader_cnt,
		     sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
		     app_reader_id_t &app_reader_cnt,
		     app_endpoint app_reader_tbl[APP_READER_MAX],
		     const config_t &conf,
		     volatile uint8_t *app_data_req,
		     volatile uint8_t *app_data_rel,
		     volatile uint8_t *app_data_grant)
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

	static uint32_t clk_cnt;

	uint8_t grant;

	if (conf.ctrl & CTRL_ENABLE) {
		if (clk_cnt == SPDP_WRITER_CYCLE) {
			SPDP_WRITER_OUT();
		} else if (clk_cnt == SEDP_PUB_WRITER_CYCLE(0)) {
			SEDP_PUB_WRITER_OUT(0);
		}else if (clk_cnt == SEDP_PUB_WRITER_CYCLE(1)) {
			SEDP_PUB_WRITER_OUT(1);
		}else if (clk_cnt == SEDP_PUB_WRITER_CYCLE(2)) {
			SEDP_PUB_WRITER_OUT(2);
		}else if (clk_cnt == SEDP_PUB_WRITER_CYCLE(3)) {
			SEDP_PUB_WRITER_OUT(3);
		} else if (clk_cnt == SEDP_SUB_WRITER_CYCLE(0)) {
			SEDP_SUB_WRITER_OUT(0);
		} else if (clk_cnt == SEDP_SUB_WRITER_CYCLE(1)) {
			SEDP_SUB_WRITER_OUT(1);
		} else if (clk_cnt == SEDP_SUB_WRITER_CYCLE(2)) {
			SEDP_SUB_WRITER_OUT(2);
		} else if (clk_cnt == SEDP_SUB_WRITER_CYCLE(3)) {
			SEDP_SUB_WRITER_OUT(3);
		} else if (clk_cnt == SEDP_PUB_HEARTBEAT_CYCLE(0)) {
			SEDP_PUB_HEARTBEAT_OUT(0);
		}else if (clk_cnt == SEDP_PUB_HEARTBEAT_CYCLE(1)) {
			SEDP_PUB_HEARTBEAT_OUT(1);
		}else if (clk_cnt == SEDP_PUB_HEARTBEAT_CYCLE(2)) {
			SEDP_PUB_HEARTBEAT_OUT(2);
		}else if (clk_cnt == SEDP_PUB_HEARTBEAT_CYCLE(3)) {
			SEDP_PUB_HEARTBEAT_OUT(3);
		} else if (clk_cnt == SEDP_SUB_HEARTBEAT_CYCLE(0)) {
			SEDP_SUB_HEARTBEAT_OUT(0);
		} else if (clk_cnt == SEDP_SUB_HEARTBEAT_CYCLE(1)) {
			SEDP_SUB_HEARTBEAT_OUT(1);
		} else if (clk_cnt == SEDP_SUB_HEARTBEAT_CYCLE(2)) {
			SEDP_SUB_HEARTBEAT_OUT(2);
		} else if (clk_cnt == SEDP_SUB_HEARTBEAT_CYCLE(3)) {
			SEDP_SUB_HEARTBEAT_OUT(3);
		} else if (clk_cnt == SEDP_PUB_ACKNACK_CYCLE(0)) {
			SEDP_PUB_ACKNACK_OUT(0);
		} else if (clk_cnt == SEDP_PUB_ACKNACK_CYCLE(1)) {
			SEDP_PUB_ACKNACK_OUT(1);
		} else if (clk_cnt == SEDP_PUB_ACKNACK_CYCLE(2)) {
			SEDP_PUB_ACKNACK_OUT(2);
		} else if (clk_cnt == SEDP_PUB_ACKNACK_CYCLE(3)) {
			SEDP_PUB_ACKNACK_OUT(3);
		} else if (clk_cnt == SEDP_SUB_ACKNACK_CYCLE(0)) {
			SEDP_SUB_ACKNACK_OUT(0);
		} else if (clk_cnt == SEDP_SUB_ACKNACK_CYCLE(1)) {
			SEDP_SUB_ACKNACK_OUT(1);
		} else if (clk_cnt == SEDP_SUB_ACKNACK_CYCLE(2)) {
			SEDP_SUB_ACKNACK_OUT(2);
		} else if (clk_cnt == SEDP_SUB_ACKNACK_CYCLE(3)) {
			SEDP_SUB_ACKNACK_OUT(3);
		} else if (clk_cnt == APP_WRITER_CYCLE(0)) {
			APP_WRITER_OUT(0);
		} else if (clk_cnt == APP_WRITER_CYCLE(1)) {
			APP_WRITER_OUT(1);
		} else if (clk_cnt == APP_WRITER_CYCLE(2)) {
			APP_WRITER_OUT(2);
		} else if (clk_cnt == APP_WRITER_CYCLE(3)) {
			APP_WRITER_OUT(3);
		}
	}

#ifdef USE_FIFOIF_ETHERNET
	tx_buf.shift_out(out);
#else // !USE_FIFOIF_ETHERNET
	tx_buf.shift_out(s);

	slip_out(s, out);
#endif // USE_FIFOIF_ETHERNET

	clk_cnt++;
	if (clk_cnt == CLOCK_COUNT_CYCLE)
		clk_cnt = 0;
}

/* Cyber func=process_pipeline */
void ros2(hls_stream<uint8_t> &in/* Cyber port_mode=cw_fifo */,
	  hls_stream<uint8_t> &out/* Cyber port_mode=cw_fifo */,
	  const config_t &conf,
	  volatile uint8_t *app_data_req,
	  volatile uint8_t *app_data_rel,
	  volatile uint8_t *app_data_grant)
{
#pragma HLS interface ap_fifo port=in
#pragma HLS interface ap_fifo port=out
#pragma HLS interface ap_none port=conf
#pragma HLS disaggregate variable=conf
#pragma HLS interface ap_vld port=app_data_req register
#pragma HLS interface ap_vld port=app_data_rel register
#pragma HLS interface ap_none port=app_data_grant
#pragma HLS interface ap_ctrl_none port=return

	static sedp_reader_id_t sedp_reader_cnt;
	static app_reader_id_t app_reader_cnt;

	static sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX];
	static app_endpoint app_reader_tbl[APP_READER_MAX];
#pragma HLS array_partition variable=sedp_reader_tbl complete dim=0
#pragma HLS array_partition variable=app_reader_tbl complete dim=0

	if (!(conf.ctrl & CTRL_ENABLE)) {
		// reset
		sedp_reader_cnt = 0;
		app_reader_cnt = 0;
	}

	ros2_in(in,
		sedp_reader_cnt,
		sedp_reader_tbl,
		app_reader_cnt,
		app_reader_tbl,
		conf);

	ros2_out(out,
		 sedp_reader_cnt,
		 sedp_reader_tbl,
		 app_reader_cnt,
		 app_reader_tbl,
		 conf,
		 app_data_req,
		 app_data_rel,
		 app_data_grant);
}
