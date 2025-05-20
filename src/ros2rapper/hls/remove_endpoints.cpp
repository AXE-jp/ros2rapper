// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

#include "remove_endpoints.hpp"

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
void update_liveliness(hls_uint<9> in, const config_t *conf,
                       sedp_endpoint sedp_reader_tbl[SEDP_READER_MAX],
                       app_endpoint  app_reader_tbl[APP_READER_MAX],
                       bool         *reading_rtps_message) {
    // 1. Change reading_rtps_message to tell whether or not the garbage
    //    collector can change the endpoint tables. When reading_rtps_message is
    //    true, the endpoint tables should not be changed.
    // 2. When the ros2rapper gets a message which tells disposed or
    //    unregistered by inline QoS, set endpoint table status dead (set the
    //    member alive false.) This module only uses GUID prefix and INFO_DST to
    //    find which endpoint dies.
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
    static hls_uint<APP_READER_MAX>  app_unmatched;

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
            compare_guid_prefix_of_app_endpoint(data, app_reader_tbl, offset,
                                                app_unmatched);
        }
        // Tell the garbage collector not to change the endpoint tables
        // because the ros2rapper uses them to process the RTPS message.
        *reading_rtps_message = true;
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
            if (conf->guid_prefix[offset] != data) {
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
            } else {
                state = STATE_SKIP_TO_NEXT_PARAM;
            }
        }
        break;
    case STATE_READ_STATUS_INFO:
        // See RTPS 2.3 specification 9.6.3.9.
        if (offset == 3) {
            bool alive = ((data & 3) == 0);
            /* Cyber unroll_times=all */
            for (auto j = 0; j < SEDP_READER_MAX; j++) {
#pragma HLS unroll
                if (!sedp_unmatched[j]) {
                    sedp_reader_tbl[j].alive = alive;
                }
            }
            /* Cyber unroll_times=all */
            for (auto j = 0; j < APP_READER_MAX; j++) {
#pragma HLS unroll
                if (!app_unmatched[j]) {
                    app_reader_tbl[j].alive = alive;
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
        app_unmatched = 0;
        offset = 0;
        state = STATE_READ_RTPS_HDR;
    }
}

/* Cyber func=inline */
void collect_dead_endpoint(hls_uint<2>       tx_progress,
                           sedp_reader_id_t &sedp_reader_cnt,
                           sedp_endpoint     sedp_reader_tbl[SEDP_READER_MAX],
                           app_reader_id_t  &app_reader_cnt,
                           app_endpoint      app_reader_tbl[APP_READER_MAX],
                           uint32_t sedp_pub_heartbeat_cnt[SEDP_READER_MAX],
                           uint32_t sedp_sub_heartbeat_cnt[SEDP_READER_MAX],
                           uint32_t sedp_pub_acknack_cnt[SEDP_READER_MAX],
                           uint32_t sedp_sub_acknack_cnt[SEDP_READER_MAX]) {
#pragma HLS inline
    constexpr uint8_t invalid_guid_prefix[GUID_PREFIX_SIZE]
        = GUID_PREFIX_UNKNOWN;
    if ((tx_progress < SEDP_READER_MAX)
        && !sedp_reader_tbl[tx_progress].alive) {
        if (((tx_progress + 1) < SEDP_READER_MAX)
            && ((tx_progress + 1) < sedp_reader_cnt)) {
            // If there is a dead sedp_endpoint in the middle of the table,
            // remove the dead sedp_endpoint and copy the next one.
            sedp_reader_tbl[tx_progress] = sedp_reader_tbl[tx_progress + 1];
            // Kill the copied sedp_endpoint.
            /* Cyber unroll_times=all */
            for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
                sedp_reader_tbl[tx_progress + 1].guid_prefix[j]
                    = invalid_guid_prefix[j];
            }
            sedp_reader_tbl[tx_progress + 1].alive = false;
            // Copy counters.
            sedp_pub_heartbeat_cnt[tx_progress]
                = sedp_pub_heartbeat_cnt[tx_progress + 1];
            sedp_sub_heartbeat_cnt[tx_progress]
                = sedp_sub_heartbeat_cnt[tx_progress + 1];
            sedp_pub_acknack_cnt[tx_progress]
                = sedp_pub_acknack_cnt[tx_progress + 1];
            sedp_sub_acknack_cnt[tx_progress]
                = sedp_sub_acknack_cnt[tx_progress + 1];
        } else if ((tx_progress + 1) == sedp_reader_cnt) {
            // If there is a dead sedp_endpoint at the end of the table
            sedp_reader_cnt--;
        }
    }
    if ((tx_progress < APP_READER_MAX) && !app_reader_tbl[tx_progress].alive) {
        if (((tx_progress + 1) < APP_READER_MAX)
            && ((tx_progress + 1) < app_reader_cnt)) {
            // If there is a dead app_endpoint in the middle of the table,
            // remove the dead app_endpoint and copy the next one.
            app_reader_tbl[tx_progress] = app_reader_tbl[tx_progress + 1];
            // Kill the copied app_endpoint.
            /* Cyber unroll_times=all */
            for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
                app_reader_tbl[tx_progress + 1].guid_prefix[j]
                    = invalid_guid_prefix[j];
            }
            app_reader_tbl[tx_progress + 1].alive = false;
        } else if ((tx_progress + 1) == app_reader_cnt) {
            // If there is a dead app_endpoint at the end of the table
            app_reader_cnt--;
        }
    }
}
