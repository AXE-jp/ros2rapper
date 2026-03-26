#include "spdp_reader.hpp"
#include "endpoint.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include "ros2_receiver.hpp"
#include "rtps.hpp"

/* Cyber func=inline */
bool spdp_set_locator(uint16_t offset, uint8_t data, bool param_le,
                      uint8_t ip_addr[4], uint8_t udp_port[2],
                      const uint8_t reader_ip_addr[4],
                      const uint8_t subnet_mask[4], uint16_t port_num_seed) {
#pragma HLS inline
    if (param_le && (offset == 4)) {
        udp_port[1] = data;
    } else if (param_le && (offset == 5)) {
        udp_port[0] = data;
    } else if (!param_le && (offset == 6)) {
        udp_port[0] = data;
    } else if (!param_le && (offset == 7)) {
        udp_port[1] = data;
    } else if (offset == 20) {
        ip_addr[0] = data;
    } else if (offset == 21) {
        ip_addr[1] = data;
    } else if (offset == 22) {
        ip_addr[2] = data;
    } else if (offset == 23) {
        ip_addr[3] = data;
    }
    if ((offset >= 23)
        && is_same_subnet(ip_addr, reader_ip_addr, subnet_mask)) {
        uint16_t port_num = (udp_port[0] << 8) | udp_port[1];
        return (port_num >= port_num_seed) && (port_num < (port_num_seed + DG));
    }
    return false;
}

/* Cyber func=inline */
static bool spdp_set_lease_duration(uint16_t offset, uint8_t data,
                                    bool param_le, uint8_t lease_duration[8]) {
#pragma HLS inline
    if (param_le) {
        if (offset == 0) {
            lease_duration[4] = data;
        } else if (offset == 1) {
            lease_duration[5] = data;
        } else if (offset == 2) {
            lease_duration[6] = data;
        } else if (offset == 3) {
            lease_duration[7] = data;
        } else if (offset == 4) {
            lease_duration[0] = data;
        } else if (offset == 5) {
            lease_duration[1] = data;
        } else if (offset == 6) {
            lease_duration[2] = data;
        } else if (offset == 7) {
            lease_duration[3] = data;
        }
    } else {
        if (offset == 0) {
            lease_duration[7] = data;
        } else if (offset == 1) {
            lease_duration[6] = data;
        } else if (offset == 2) {
            lease_duration[5] = data;
        } else if (offset == 3) {
            lease_duration[4] = data;
        } else if (offset == 4) {
            lease_duration[3] = data;
        } else if (offset == 5) {
            lease_duration[2] = data;
        } else if (offset == 6) {
            lease_duration[1] = data;
        } else if (offset == 7) {
            lease_duration[0] = data;
        }
    }
    return (offset >= 7);
}

/* Cyber func=inline */
static void send_received_spdp(hls_stream<rtps_data_t> &out,
                               const uint8_t src_guid_prefix[GUID_PREFIX_SIZE],
                               const uint8_t spdp_ip_addr[4],
                               const uint8_t spdp_udp_port[2],
                               bool          lease_duration_found,
                               const uint8_t spdp_lease_duration[8]) {
#pragma HLS inline
    rtps_data_t rtps_data;
#pragma HLS array_partition variable = rtps_data.guid_prefix complete dim = 1
#pragma HLS array_partition variable = rtps_data.data complete dim = 1
    rtps_data.type = RTPS_TYPE_SPDP;
    /* Cyber unroll_times=all */
    for (auto j = 0; j < GUID_PREFIX_SIZE; j++) {
#pragma HLS unroll
        rtps_data.guid_prefix[j] = src_guid_prefix[j];
    }
    rtps_data.data[0] = spdp_ip_addr[0];
    rtps_data.data[1] = spdp_ip_addr[1];
    rtps_data.data[2] = spdp_ip_addr[2];
    rtps_data.data[3] = spdp_ip_addr[3];
    rtps_data.data[4] = spdp_udp_port[0];
    rtps_data.data[5] = spdp_udp_port[1];
    if (lease_duration_found) {
        /* Cyber unroll_times=all */
        for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
            rtps_data.data[j + 6] = spdp_lease_duration[j];
        }
    } else {
        // Use default value 100 sec.
        rtps_data.data[6] = 0;
        rtps_data.data[7] = 0;
        rtps_data.data[8] = 0;
        rtps_data.data[9] = 0;
        rtps_data.data[10] = 100;
        rtps_data.data[11] = 0;
        rtps_data.data[12] = 0;
        rtps_data.data[13] = 0;
    }
    out.write(rtps_data);
}

typedef enum {
    SPDP_READER_STATE_PAYLOAD_HDR,
    SPDP_READER_STATE_PARAM_HDR,
    SPDP_READER_STATE_PARAM_PAYLOAD,
    SPDP_READER_STATE_WAIT_END
} spdp_reader_state_t;

/* Cyber func=inline */
void spdp_reader(hls_uint<9> x, hls_stream<rtps_data_t> &out,
                 const uint8_t reader_ip_addr[4], const uint8_t subnet_mask[4],
                 uint16_t      port_num_seed,
                 const uint8_t src_guid_prefix[GUID_PREFIX_SIZE]) {
#pragma HLS inline
    static spdp_reader_state_t state = SPDP_READER_STATE_PAYLOAD_HDR;
    static uint16_t            offset = 0;
    static uint16_t            rep_id;
    static uint16_t            param_id;
    static uint16_t            param_length;
    bool                       param_le = (rep_id & SP_ID_CDR_LE);

    static bool locator_found;
    static bool lease_duration_found;

    static uint8_t spdp_ip_addr[4] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = spdp_ip_addr complete dim = 1
    static uint8_t spdp_udp_port[2] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = spdp_udp_port complete dim = 1
    static uint8_t spdp_lease_duration[8] /* Cyber array=EXPAND */;
#pragma HLS array_partition variable = spdp_lease_duration complete dim = 1

    uint8_t data = x & 0xff;
    bool    end = x & 0x100;

    switch (state) {
    case SPDP_READER_STATE_PAYLOAD_HDR:
        switch (offset) {
        case SP_HDR_OFFSET_REPRESENTATION_ID:
            rep_id = data << 8;
            break;
        case SP_HDR_OFFSET_REPRESENTATION_ID + 1:
            rep_id |= data;
            break;
        }
        offset++;
        if (offset == SP_HDR_SIZE) {
            offset = 0;
            locator_found = false;
            lease_duration_found = false;
            if (rep_id & SP_ID_PL_CDR) {
                state = SPDP_READER_STATE_PARAM_HDR;
            } else {
                state = SPDP_READER_STATE_WAIT_END;
            }
        }
        break;
    case SPDP_READER_STATE_PARAM_HDR:
        switch (offset) {
        case 0:
            param_id = param_le ? data : (data << 8);
            break;
        case 1:
            param_id |= param_le ? (data << 8) : data;
            break;
        case 2:
            param_length = param_le ? data : (data << 8);
            break;
        case 3:
            param_length |= param_le ? (data << 8) : data;
            break;
        }
        offset++;
        if (offset == 4) {
            offset = 0;
            if (param_id == PID_SENTINEL) {
                if (locator_found) {
                    send_received_spdp(out, src_guid_prefix, spdp_ip_addr,
                                       spdp_udp_port, lease_duration_found,
                                       spdp_lease_duration);
                }
                state = SPDP_READER_STATE_WAIT_END;
            } else if (param_length == 0) {
                state = SPDP_READER_STATE_PARAM_HDR;
            } else {
                state = SPDP_READER_STATE_PARAM_PAYLOAD;
            }
        }
        break;
    case SPDP_READER_STATE_PARAM_PAYLOAD:
        if (!locator_found && (param_id == PID_METATRAFFIC_UNICAST_LOCATOR)) {
            locator_found = spdp_set_locator(
                offset, data, param_le, spdp_ip_addr, spdp_udp_port,
                reader_ip_addr, subnet_mask, port_num_seed);
        } else if (!lease_duration_found
                   && (param_id == PID_PARTICIPANT_LEASE_DURATION)) {
            lease_duration_found = spdp_set_lease_duration(
                offset, data, param_le, spdp_lease_duration);
        }
        offset++;
        if (offset == param_length) {
            offset = 0;
            state = SPDP_READER_STATE_PARAM_HDR;
        }
        break;
    case SPDP_READER_STATE_WAIT_END:
        break;
    }

    if (end) {
        state = SPDP_READER_STATE_PAYLOAD_HDR;
        offset = 0;
    }
}
