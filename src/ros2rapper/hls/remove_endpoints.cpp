// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "remove_endpoints.hpp"

/* Cyber func=inline */
static void remove_sedp_endpoint(sedp_reader_id_t id,
                                 sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
                                 app_endpoint  app_reader_tbl[APP_READER_MAX]) {
#pragma HLS inline
    if (id < SEDP_READER_MAX) {
        // Remove sedp_reader_tbl[id].
        sedp_reader_tbl[id].alive = false;
        // Remove the children of sedp_reader_tbl[id].
        /* Cyber unroll_times=all */
        for (auto j = 0; j < APP_READER_MAX; j++) {
#pragma HLS unroll
            if (sedp_reader_tbl[id].children[j]) {
                app_reader_tbl[j].alive = false;
            }
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
void update_liveliness(hls_uint<9>   in,
                       const uint8_t reader_guid_prefix[GUID_PREFIX_SIZE],
                       sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
                       app_endpoint  app_reader_tbl[APP_READER_MAX],
                       bool *reading_rtps_message, int64_t timestamp_i64) {
    // 1. Change reading_rtps_message to tell whether or not the garbage
    //    collector can change the endpoint tables. When reading_rtps_message is
    //    true, the endpoint tables should not be changed.
    // 2. When the ros2rapper gets a message which tells disposed or
    //    unregistered, remove (i.e. set the member '.alive' false) the endpoint
    //    which sent the message.
    // 3. When the ros2rapper gets a message from a known participant, update
    //    timestamp of its data.
#pragma HLS inline
    static update_liveliness_state_t state;
    static uint16_t                  offset;

    static uint8_t  sbm_id;
    static bool     sbm_le;
    static bool     sbm_inline_qos;
    static uint16_t sbm_len;

    static uint16_t param_id;
    static uint16_t param_len;

    static hls_uint<SEDP_READER_MAX> sedp_unmatched;

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
            compare_guid_prefix_of_sedp_endpoint(data, sedp_reader_tbl, offset,
                                                 sedp_unmatched);
        }
        // Tell the garbage collector not to change the endpoint tables
        // because the ros2rapper uses them to process a RTPS message.
        *reading_rtps_message = true;
        offset++;
        if (offset == GUID_PREFIX_SIZE) {
            offset = 0;
            state = STATE_READ_SBM_HDR;
            // Update timestamps of matched endpoints in sedp_reader_tbl.
            /* Cyber unroll_times=all */
            for (auto j = 0; j < SEDP_READER_MAX; j++) {
#pragma HLS unroll
                if (!sedp_unmatched[j]) {
                    sedp_reader_tbl[j].timestamp = timestamp_i64;
                }
            }
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
                /* Cyber unroll_times=all */
                for (auto j = 0; j < SEDP_READER_MAX; j++) {
#pragma HLS unroll
                    if (!sedp_unmatched[j]) {
                        remove_sedp_endpoint(j, sedp_reader_tbl,
                                             app_reader_tbl);
                    }
                }
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
        // Allow the garbage collector to change the endpoint tables.
        *reading_rtps_message = false;
        sedp_unmatched = 0;
        offset = 0;
        state = STATE_READ_RTPS_HDR;
    }
}

/* Cyber func=inline */
void remove_dead_endpoints(sedp_reader_id_t id,
                           sedp_endpoint    sedp_reader_tbl[SEDP_READER_MAX],
                           app_endpoint     app_reader_tbl[APP_READER_MAX],
                           int64_t          timestamp_i64) {
#pragma HLS inline
    // Check timeout
    if (id < SEDP_READER_MAX) {
        if ((timestamp_i64 - sedp_reader_tbl[id].timestamp)
            > sedp_reader_tbl[id].lease_duration) {
            remove_sedp_endpoint(id, sedp_reader_tbl, app_reader_tbl);
        }
    }
}
