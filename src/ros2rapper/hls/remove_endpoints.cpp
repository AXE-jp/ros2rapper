// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "remove_endpoints.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include <cstdint>

/* Cyber func=inline */
void remove_sedp_endpoint(sedp_reader_id_t   sedp_idx,
                          sedp_reader_tbl_t *sedp_reader_tbl,
                          app_reader_tbl_t  *app_reader_tbl,
                          sedp_reader_id_t  *sedp_reader_cnt,
                          app_reader_id_t   *app_reader_cnt) {
#pragma HLS inline
    uint64_t children_0, children_1;
    get_sedp_reader_tbl(&children_0, sedp_reader_tbl, sedp_idx, 9);
    get_sedp_reader_tbl(&children_1, sedp_reader_tbl, sedp_idx, 10);

    // Remove sedp_endpoint
    set_sedp_reader_tbl(0, sedp_reader_tbl, sedp_idx, 0);
    (*sedp_reader_cnt)--;

    // Remove the children of the sedp_endpoint
#ifdef APP_READER_TBL_FF
    /* Cyber unroll_times=all */
#else
    /* Cyber folding=1 */
#endif
    for (auto j = 0; j < 64; j++) {
#ifdef APP_READER_TBL_FF
#pragma HLS unroll
#else
#pragma HLS pipeline II = 1
#endif
        uint64_t flag = static_cast<uint64_t>(1) << j;
        if ((children_0 & flag) != 0) {
            app_reader_tbl->ram[j] = 0;
            (*app_reader_cnt)--;
        }
    }

#ifdef APP_READER_TBL_FF
    /* Cyber unroll_times=all */
#else
    /* Cyber folding=1 */
#endif
    for (auto j = 0; j < 64; j++) {
#ifdef APP_READER_TBL_FF
#pragma HLS unroll
#else
#pragma HLS pipeline II = 1
#endif
        uint64_t flag = static_cast<uint64_t>(1) << j;
        if ((children_0 & flag) != 0) {
            app_reader_tbl->ram[j + 64] = 0;
            (*app_reader_cnt)--;
        }
    }
}

typedef enum {
    STATE_READ_RTPS_HDR,
    STATE_READ_HDR_GUID_PREFIX,
    STATE_READ_SBM_HDR,
    STATE_READ_INFO_DST,
    STATE_SKIP_EXTRA_FLAGS,
    STATE_READ_OCTETS_TO_INLINE_QOS,
    STATE_SKIP_TO_NEXT_PARAM,
    STATE_READ_PARAM_ID,
    STATE_READ_PARAM_LEN,
    STATE_READ_STATUS_INFO,
    STATE_SKIP_SBM,
    STATE_WAIT_END
} update_liveliness_state_t;

/* Cyber func=inline */
void update_liveliness(hls_uint<9> in, hls_stream<rtps_data_t> &out,
                       const uint8_t reader_guid_prefix[GUID_PREFIX_SIZE]) {
    // When the ros2rapper gets a message which tells disposed or unregistered,
    // remove (i.e. set the member '.alive' false) the endpoint which sent the
    // message.
#pragma HLS inline
    static update_liveliness_state_t state;
    static uint16_t                  offset;

    static uint8_t  sbm_id;
    static bool     sbm_le;
    static bool     sbm_inline_qos;
    static uint16_t sbm_len;

    static uint16_t param_id;
    static uint16_t param_len;

    static uint8_t inline_qos_guid_prefix
        [GUID_PREFIX_SIZE] /* Cyber array=EXPAND, array_index=const */;
#pragma HLS array_partition variable = inline_qos_guid_prefix complete dim = 1

    uint8_t data = in & 0xff;
    bool    end = in & 0x100;

    switch (state) {
    case STATE_READ_RTPS_HDR:
        if (!rtps_compare_protocol(offset, data)) {
            state = STATE_WAIT_END;
            break;
        }
        offset++;
        if (offset == RTPS_HDR_OFFSET_GUID_PREFIX) {
            offset = 0;
            state = STATE_READ_HDR_GUID_PREFIX;
        }
        break;
    case STATE_READ_HDR_GUID_PREFIX:
        if (offset < GUID_PREFIX_SIZE) {
            inline_qos_guid_prefix[offset] = data;
        }
        // Tell the garbage collector not to change the endpoint tables
        // because the ros2rapper uses them to process a RTPS message.
        offset++;
        if (offset == GUID_PREFIX_SIZE) {
            offset = 0;
            state = STATE_READ_SBM_HDR;
        }
        break;
    case STATE_READ_SBM_HDR:
        switch (offset) {
        case SBM_HDR_OFFSET_SUBMESSAGE_ID:
            sbm_id = data;
            break;
        case SBM_HDR_OFFSET_FLAGS:
            sbm_le = (data & SBM_FLAGS_ENDIANNESS) ? true : false;
            sbm_inline_qos = (data & SBM_FLAGS_INLINE_QOS) ? true : false;
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER:
            sbm_len = sbm_le ? data : (data << 8);
            break;
        case SBM_HDR_OFFSET_OCTETS_TO_NEXT_HEADER + 1:
            sbm_len |= sbm_le ? (data << 8) : data;
        }
        offset++;
        if (offset == SBM_HDR_SIZE) {
            offset = 0;
            if (sbm_id == SBM_ID_INFO_DST) {
                state = STATE_READ_INFO_DST;
            } else if ((sbm_id == SBM_ID_DATA) && sbm_inline_qos) {
                // PID_STATUS_INFO only appears in inline QoS
                // (see RTPS 2.3 specification 9.6.3.)
                state = STATE_SKIP_EXTRA_FLAGS;
            } else {
                state = STATE_SKIP_SBM;
            }
        }
        break;
    case STATE_READ_INFO_DST:
        if (offset < GUID_PREFIX_SIZE) {
            if (reader_guid_prefix[offset] != data) {
                state = STATE_WAIT_END;
                break;
            }
        }
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = STATE_READ_SBM_HDR;
        }
        break;
    case STATE_SKIP_EXTRA_FLAGS:
        offset++;
        if (offset == SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS) {
            offset = 0;
            sbm_le -= SBM_DATA_HDR_OFFSET_OCTETS_TO_INLINE_QOS;
            state = STATE_READ_OCTETS_TO_INLINE_QOS;
        }
        break;
    case STATE_READ_OCTETS_TO_INLINE_QOS:
        if (offset == 0) {
            param_len = sbm_le ? data : (data << 8);
        } else {
            param_len |= sbm_le ? (data << 8) : data;
        }
        offset++;
        if (offset == sizeof(param_len)) {
            offset = 0;
            sbm_len -= sizeof(param_len);
            state = STATE_SKIP_TO_NEXT_PARAM;
        }
        break;
    case STATE_SKIP_TO_NEXT_PARAM:
        offset++;
        if (offset == param_len) {
            offset = 0;
            sbm_len -= param_len;
            state = STATE_READ_PARAM_ID;
        }
        break;
    case STATE_READ_PARAM_ID:
        if (offset == 0) {
            param_id = sbm_le ? data : (data << 8);
        } else {
            param_id |= sbm_le ? (data << 8) : data;
        }
        offset++;
        if (offset == sizeof(param_id)) {
            offset = 0;
            sbm_len -= sizeof(param_id);
            state = STATE_READ_PARAM_LEN;
        }
        break;
    case STATE_READ_PARAM_LEN:
        if (offset == 0) {
            param_len = sbm_le ? data : (data << 8);
        } else {
            param_len |= sbm_le ? (data << 8) : data;
        }
        offset++;
        if (offset == sizeof(param_len)) {
            offset = 0;
            sbm_len -= sizeof(param_len);
            if (param_id == PID_SENTINEL) {
                if (sbm_len > 0) {
                    state = STATE_SKIP_SBM;
                } else {
                    state = STATE_READ_SBM_HDR;
                }
            } else if (param_id == PID_STATUS_INFO) {
                state = STATE_READ_STATUS_INFO;
            } else if (param_len == 0) {
                state = STATE_READ_PARAM_ID;
            } else {
                state = STATE_SKIP_TO_NEXT_PARAM;
            }
        }
        break;
    case STATE_READ_STATUS_INFO:
        // See RTPS 2.3 specification 9.6.3.9.
        if (offset == 3) {
            if ((data & 3) != 0) {
                // disposed (0x01) or unregistered (0x02)
                rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
                rtps_data.type = RTPS_TYPE_RM_ENDPOINT;
                /* Cyber unroll_times=all */
                for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
                    rtps_data.guid_prefix[j] = inline_qos_guid_prefix[j];
                }
                out.write(rtps_data);
            }
        }
        offset++;
        if (offset == PID_STATUS_INFO_SIZE) {
            offset = 0;
            sbm_len -= PID_STATUS_INFO_SIZE;
            state = STATE_READ_PARAM_ID;
        }
        break;
    case STATE_SKIP_SBM:
        offset++;
        if (offset == sbm_len) {
            offset = 0;
            state = STATE_READ_SBM_HDR;
        }
        break;
    default:;
    }

    if (end) {
        offset = 0;
        state = STATE_READ_RTPS_HDR;
    }
}

/* Cyber func=inline */
void remove_dead_endpoints(sedp_reader_id_t   id,
                           sedp_reader_tbl_t *sedp_reader_tbl,
                           app_reader_tbl_t  *app_reader_tbl,
                           int64_t            timestamp_i64,
                           sedp_reader_id_t  *sedp_reader_cnt,
                           app_reader_id_t   *app_reader_cnt) {
#pragma HLS inline
    // Check timeout
    if (id < SEDP_READER_MAX) {
        uint64_t flags;
        get_sedp_reader_tbl(&flags, sedp_reader_tbl, id, 0);
        if ((flags & SEDP_ENDPOINT_ALIVE) != 0) {
            int64_t lease_duration, last_spdp_timestamp;
            get_sedp_reader_tbl_lease_duration(&lease_duration, sedp_reader_tbl,
                                               id);
            get_sedp_reader_tbl_timestamp(&last_spdp_timestamp, sedp_reader_tbl,
                                          id);
            if ((timestamp_i64 - last_spdp_timestamp) > lease_duration) {
                remove_sedp_endpoint(id, sedp_reader_tbl, app_reader_tbl,
                                     sedp_reader_cnt, app_reader_cnt);
            }
        }
    }
}
