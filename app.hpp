#ifndef APP_HPP
#define APP_HPP

#include <cstdint>
#include "rtps.hpp"
#include "timestamp.hpp"

#define APP_OCTETS_TO_NEXT_HEADER(len)				\
	(SBM_DATA_HDR_SIZE + SP_HDR_SIZE + SP_DATA_SIZE(len))

#define APP_TOT_LEN(len)					\
	(RTPS_HDR_SIZE + SBM_HDR_SIZE + GUID_PREFIX_SIZE +	\
	 SBM_HDR_SIZE + TIMESTAMP_SIZE +			\
	 SBM_HDR_SIZE + APP_OCTETS_TO_NEXT_HEADER(len))

void app_writer(const uint8_t writer_guid_prefix[12],
		const uint8_t writer_entity_id[4],
		const uint8_t reader_guid_prefix[12],
		const uint8_t reader_entity_id[4],
		const int64_t seqnum,
		const uint8_t app_data[],
		const uint32_t app_data_len,
		uint8_t buf[]);

#endif // !APP_HPP
