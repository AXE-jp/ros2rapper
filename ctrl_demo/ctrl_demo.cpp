



#include <cstdint>
#include <ap_int.h>
#include <hls_stream.h>




typedef struct {
	uint8_t		node_name[32];
	uint8_t		node_name_len;
	uint16_t	node_udp_port;
	uint16_t	cpu_udp_port;
	uint16_t	port_num_seed;
	uint32_t	tx_period;
	uint32_t	fragment_expiration;
	uint8_t		guid_prefix[12];
	uint8_t		pub_topic_name[32];
	uint8_t		pub_topic_name_len;
	uint8_t		pub_topic_type_name[64];
	uint8_t		pub_topic_type_name_len;
	uint8_t		sub_topic_name[32];
	uint8_t		sub_topic_name_len;
	uint8_t		sub_topic_type_name[64];
	uint8_t		sub_topic_type_name_len;
} ros2_conf_t;


typedef struct {
	uint8_t	data[64];
} app_data_t;


/*
 * top function
 */
void  ros2_ether_ctrl_demo(
	/************ interfaces for ros2rapper-ether ***************/
	/* configurations */
	ros2_conf_t&		ros2_conf,

	/* application data in/out */
	hls::stream<app_data_t>	&ros2_pub_data,
	uint8_t&		ros2_pub_data_len,
	uint8_t		ros2_sub_data[64],
	uint8_t		ros2_sub_data_len,
	uint8_t&	ros2_sub_data_max_len,

	/*********** application specific ports ************/
	ap_uint<4>	sw,
	ap_uint<4>&	led
)
{
#pragma HLS interface ap_ctrl_none port=return

#pragma HLS interface ap_none port=ros2_conf
#pragma HLS disaggregate variable=ros2_conf

#pragma HLS interface axis port=ros2_pub_data
#pragma HLS interface ap_none port=ros2_pub_data_len

#pragma HLS interface ap_none port=ros2_sub_data
#pragma HLS array_reshape variable=ros2_sub_data complete dim=0
#pragma HLS interface ap_none port=ros2_sub_data_len
#pragma HLS interface ap_none port=ros2_sub_data_max_len

	static const ros2_conf_t	conf0 = {
		.node_name = "talker_listener",
		.node_name_len = 16,
		.node_udp_port = 52000,
		.cpu_udp_port = 1234,
		.port_num_seed = 7400,
		.tx_period = 12500000,
		.fragment_expiration = 3333333333,
		.guid_prefix = { 0x01, 0x0f, 0x37, 0xad,
				 0xde, 0x09, 0x00, 0x00,
				 0x01, 0x00, 0x00, 0x00 },
		.pub_topic_name = "rt/chatter",
		.pub_topic_name_len = 11,
		.pub_topic_type_name = "std_msgs::msg::dds_::String_",
		.pub_topic_type_name_len = 29,
		.sub_topic_name = "rt/chatter",
		.sub_topic_name_len = 11,
		.sub_topic_type_name = "std_msgs::msg::dds_::String_",
		.sub_topic_type_name_len = 29,
	};
	ros2_conf = conf0;
	ros2_sub_data_max_len = 64;

#pragma HLS interface ap_none port=sw
#pragma HLS interface ap_none port=led

	const uint8_t	pub_msgs[4][64] = {
				"Hello, world",
				"I'm ROS2rapper.",
				"Built at " __DATE__ " " __TIME__,
				"No more to say"
			};

	uint8_t	ix;

	ix = 0;
	if (sw & (ap_uint<4>)0x01 ) {
		ix = 0;
	} else{
		if (sw & (ap_uint<4>)0x02 ) {
			ix = 1;
		} else{
			if (sw & (ap_uint<4>)0x04 ) {
				ix = 2;
			} else{
				if (sw & (ap_uint<4>)0x08 ) {
					ix = 3;
				}
			}
		}
	}

	uint8_t	len0 = 0;
	for(int ii = 0 ; ii < 64 ; ii++ ){
//#pragma UNROLL
		if (pub_msgs[ix][ii])
			len0++;
		else
			break;
	}
	ros2_pub_data_len = len0;

	app_data_t	pub_data_reg;
	for(int ii = 0 ; ii < 64 ; ii++ ){
//#pragma UNROLL
		pub_data_reg.data[ii] = pub_msgs[ix][ii];
	}
	ros2_pub_data.write( pub_data_reg );

	ap_uint<4> led_dat = 0;
	if (ros2_sub_data_len >= 2) {
		led_dat = ros2_sub_data[ros2_sub_data_len - 2];
	}
	led = led_dat;
}
