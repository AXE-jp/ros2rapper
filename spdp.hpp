#ifndef SPDP_HPP
#define SPDP_HPP

#include <cstdint>
#include "endpoint.hpp"
#include "hls.hpp"
#include "rtps.hpp"
#include "timestamp.hpp"

#define SPDP_DATA_SIZE	168

#define SPDP_WRITER_OCTETS_TO_NEXT_HEADER			\
	(SBM_DATA_HDR_SIZE + SP_HDR_SIZE + SPDP_DATA_SIZE)

#define SPDP_WRITER_TOT_LEN					\
	(RTPS_HDR_SIZE + SBM_HDR_SIZE + TIMESTAMP_SIZE +	\
	 SBM_HDR_SIZE + SPDP_WRITER_OCTETS_TO_NEXT_HEADER)

void spdp_reader(hls_stream<hls_uint<9>> &in,
		 sedp_reader_id_t &reader_cnt,
		 sedp_endpoint reader_tbl[SEDP_READER_MAX]);
void spdp_writer(const uint8_t writer_guid_prefix[12],
		 const uint8_t metatraffic_addr[4],
		 const uint8_t metatraffic_port[2],
		 const uint8_t default_addr[4],
		 const uint8_t default_port[2],
		 uint8_t buf[SPDP_WRITER_TOT_LEN]);

#endif // !SPDP_HPP
