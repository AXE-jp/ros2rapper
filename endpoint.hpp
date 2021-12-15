#ifndef ENDPOINT_HPP
#define ENDPOINT_HPP

#include <cstdint>
#include "hls.hpp"

#define SEDP_READER_MAX	4

typedef hls_uint<3> sedp_reader_id_t;

struct sedp_endpoint {
	uint8_t ip_addr[4]/* Cyber array=REG */;
	uint8_t udp_port[2]/* Cyber array=REG */;
	uint8_t guid_prefix[12]/* Cyber array=REG */;
};

#define APP_READER_MAX	4

typedef hls_uint<3> app_reader_id_t;

struct app_endpoint {
	uint8_t ip_addr[4]/* Cyber array=REG */;
	uint8_t udp_port[2]/* Cyber array=REG */;
	uint8_t guid_prefix[12]/* Cyber array=REG */;
	uint8_t entity_id[4]/* Cyber array=REG */;
};

#endif // !ENDPOINT_HPP
