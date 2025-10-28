// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "remove_endpoints.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ros2_receiver.hpp"
#include <cstdint>

/* Cyber func=inline */
static void remove_sedp_endpoint(sedp_endpoint *sedp_reader,
                                 app_endpoint app_reader_tbl[APP_READER_MAX]) {
#pragma HLS inline
#pragma HLS array_partition variable = sedp_reader->children complete dim = 1
    // Remove sedp_reader
    sedp_reader->alive = false;
    // Remove the children of sedp_reader
    /* Cyber unroll_times=all */
    for (auto j = 0; j < APP_READER_MAX; j++) {
#pragma HLS unroll
        if (sedp_reader->children[j]) {
            app_reader_tbl[j].alive = false;
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
            if (is_participant_matched) {
                sedp_endpoint reader = sedp_reader_tbl[sedp_matched_idx];
                remove_sedp_endpoint(&reader, app_reader_tbl);
                sedp_reader_tbl[sedp_matched_idx] = reader;
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
void remove_dead_endpoints(sedp_reader_id_t id,
                           sedp_endpoint    sedp_reader_tbl[SEDP_READER_MAX],
                           app_endpoint     app_reader_tbl[APP_READER_MAX],
                           int64_t          timestamp_i64) {
#pragma HLS inline
    // Check timeout
    if (id < SEDP_READER_MAX) {
        sedp_endpoint reader = sedp_reader_tbl[id];
        if (reader.alive
            && ((timestamp_i64 - reader.timestamp) > reader.lease_duration)) {
            remove_sedp_endpoint(&reader, app_reader_tbl);
            sedp_reader_tbl[id] = reader;
        }
    }
}
