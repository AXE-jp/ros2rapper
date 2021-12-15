#ifndef SEDP_HPP
#define SEDP_HPP

#include <cstdint>
#include "endpoint.hpp"
#include "hls.hpp"
#include "rtps.hpp"
#include "timestamp.hpp"

#define SEDP_DATA_SIZE	276

#define SEDP_WRITER_OCTETS_TO_NEXT_HEADER			\
	(SBM_DATA_HDR_SIZE + SP_HDR_SIZE + SEDP_DATA_SIZE)

#define SEDP_WRITER_TOT_LEN					\
	(RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE +	\
	 SBM_HDR_SIZE + TIMESTAMP_SIZE +			\
	 SBM_HDR_SIZE + SEDP_WRITER_OCTETS_TO_NEXT_HEADER)

#define SEDP_HEARTBEAT_TOT_LEN					\
	(RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE +	\
	 SBM_HDR_SIZE + SBM_HEARTBEAT_DATA_SIZE)

#define SEDP_ACKNACK_TOT_LEN					\
	(RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE +	\
	 SBM_HDR_SIZE + SBM_ACKNACK_DATA_SIZE)

void sedp_reader(hls_stream<hls_uint<9>> &in,
		 app_reader_id_t &reader_cnt,
		 app_endpoint reader_tbl[APP_READER_MAX]);
void sedp_writer(const uint8_t writer_guid_prefix[12],
		 const uint8_t writer_entity_id[4],
		 const uint8_t reader_guid_prefix[12],
		 const uint8_t reader_entity_id[4],
		 const uint8_t usertraffic_addr[4],
		 const uint8_t usertraffic_port[2],
		 const uint8_t app_entity_id[4],
		 uint8_t buf[SEDP_WRITER_TOT_LEN]);
void sedp_heartbeat(const uint8_t writer_guid_prefix[12],
		    const uint8_t writer_entity_id[4],
		    const uint8_t reader_guid_prefix[12],
		    const uint8_t reader_entity_id[4],
		    const int64_t first_seqnum,
		    const int64_t last_seqnum,
		    const uint32_t cnt,
		    uint8_t buf[SEDP_HEARTBEAT_TOT_LEN]);
void sedp_acknack(const uint8_t writer_guid_prefix[12],
		  const uint8_t writer_entity_id[4],
		  const uint8_t reader_guid_prefix[12],
		  const uint8_t reader_entity_id[4],
		  const int64_t bitmap_base,
		  const uint32_t num_bits,
		  const uint8_t bitmap[4],
		  const uint32_t cnt,
		  uint8_t buf[SEDP_ACKNACK_TOT_LEN]);

#endif // !SEDP_HPP
