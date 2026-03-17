#include "spdp_reader.hpp"
#include "hls.hpp"
#include "ip.hpp"
#include "rtps.hpp"
#include <cstdint>

static bool is_valid_udp_port(const uint8_t udp_port[2],
                              uint16_t      port_num_seed) {
#pragma HLS inline
    uint16_t port_num = (udp_port[0] << 8) | udp_port[1];
    return (port_num >= port_num_seed) && (port_num < (port_num_seed + DG));
}

#define SPDP_READER_STATE_SBM_DATA_HDR  0
#define SPDP_READER_STATE_PAYLOAD_HDR   1
#define SPDP_READER_STATE_PARAM_HDR     2
#define SPDP_READER_STATE_PARAM_PAYLOAD 3
#define SPDP_READER_STATE_WAIT_END      4

void spdp_reader(hls_stream<hls_uint<10>> &in, hls_stream<spdp_reader_t> &out,
                 const uint8_t reader_ip_addr[4], const uint8_t subnet_mask[4],
                 uint16_t port_num_seed) {
#pragma HLS interface mode = ap_ctrl_none port = return
#pragma HLS interface mode = axis port = in
#pragma HLS interface mode = axis port = out
#pragma HLS array_reshape variable = reader_ip_addr type = complete dim = 0
#pragma HLS interface mode = ap_none port = reader_ip_addr
#pragma HLS array_reshape variable = subnet_mask type = complete dim = 0
#pragma HLS interface mode = ap_none port = subnet_mask
#pragma HLS interface mode = ap_none port = port_num_seed
    static const uint8_t par_reader_id[4] /* Cyber array=EXPAND */
        = ENTITYID_BUILTIN_PARTICIPANT_READER;
#pragma HLS array_partition variable = par_reader_id complete dim = 0

    static hls_uint<3> state = SPDP_READER_STATE_SBM_DATA_HDR;
    static uint16_t    offset = 0;

    static uint16_t rep_id;
    static uint16_t param_id;
    static uint16_t param_length;
    bool            param_le = (rep_id == SP_ID_PL_CDR_LE);

    static bool locator_found;
    static bool lease_duration_found;

    static uint8_t ip_addr[4];
#pragma HLS array_partition variable = ip_addr type = complete dim = 1
    static uint8_t udp_port[2];
#pragma HLS array_partition variable = udp_port type = complete dim = 1
    static uint8_t lease_duration[8];
#pragma HLS array_partition variable = lease_duration type = complete dim = 1

    hls_uint<10> x = in.read();
    uint8_t      data = x & 0xff;
    bool         end = x & 0x100;
    bool         valid = x & 0x200;

    spdp_reader_t out_data;
#pragma HLS array_partition variable = out_data.ip_addr type = complete dim = 1
#pragma HLS array_partition variable = out_data.udp_port type = complete dim = 1
#pragma HLS array_partition variable = out_data.lease_duration type            \
    = complete                                                 dim = 1
    out_data.valid = 0;
    // The default participant lease duration is 100 sec.
    out_data.lease_duration[0] = 0;
    out_data.lease_duration[1] = 0;
    out_data.lease_duration[2] = 0;
    out_data.lease_duration[3] = 0;
    out_data.lease_duration[4] = 100;
    out_data.lease_duration[5] = 0;
    out_data.lease_duration[6] = 0;
    out_data.lease_duration[7] = 0;

    if (valid) {
        switch (state) {
        case SPDP_READER_STATE_SBM_DATA_HDR:
            if (!rtps_compare_data_hdr_reader_id(offset, data, par_reader_id)) {
                state = SPDP_READER_STATE_WAIT_END;
                break;
            }
            offset++;
            if (offset == SBM_DATA_HDR_SIZE) {
                offset = 0;
                state = SPDP_READER_STATE_PAYLOAD_HDR;
            }
            break;
        case SPDP_READER_STATE_PAYLOAD_HDR:
            switch (offset) {
            case SP_HDR_OFFSET_REPRESENTATION_ID:
                rep_id = data << 8;
                break;
            case SP_HDR_OFFSET_REPRESENTATION_ID + 1:
                rep_id |= data;
            }
            offset++;
            if (offset == SP_HDR_SIZE) {
                offset = 0;
                locator_found = false;
                lease_duration_found = false;
                if (rep_id == SP_ID_PL_CDR_BE) {
                    state = SPDP_READER_STATE_PARAM_HDR;
                } else if (rep_id == SP_ID_PL_CDR_LE) {
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
                    out_data.valid = locator_found;
                    if (lease_duration_found) {
                        for (auto j = 0; j < 8; j++) {
#pragma HLS unroll
                            out_data.lease_duration[j] = lease_duration[j];
                        }
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
            if (!locator_found
                && (param_id == PID_METATRAFFIC_UNICAST_LOCATOR)) {
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
                    if (is_same_subnet(ip_addr, reader_ip_addr, subnet_mask)
                        && is_valid_udp_port(udp_port, port_num_seed)) {
                        locator_found = true;
                    }
                }
            } else if (!lease_duration_found
                       && (param_id == PID_PARTICIPANT_LEASE_DURATION)) {
                if (offset == 0) {
                    if (param_le) {
                        lease_duration[4] = data;
                    } else {
                        lease_duration[7] = data;
                    }
                } else if (offset == 1) {
                    if (param_le) {
                        lease_duration[5] = data;
                    } else {
                        lease_duration[6] = data;
                    }
                } else if (offset == 2) {
                    if (param_le) {
                        lease_duration[6] = data;
                    } else {
                        lease_duration[5] = data;
                    }
                } else if (offset == 3) {
                    if (param_le) {
                        lease_duration[5] = data;
                    } else {
                        lease_duration[4] = data;
                    }
                } else if (offset == 4) {
                    if (param_le) {
                        lease_duration[0] = data;
                    } else {
                        lease_duration[3] = data;
                    }
                } else if (offset == 5) {
                    if (param_le) {
                        lease_duration[1] = data;
                    } else {
                        lease_duration[2] = data;
                    }
                } else if (offset == 6) {
                    if (param_le) {
                        lease_duration[2] = data;
                    } else {
                        lease_duration[1] = data;
                    }
                } else if (offset == 7) {
                    if (param_le) {
                        lease_duration[3] = data;
                    } else {
                        lease_duration[0] = data;
                    }
                    lease_duration_found = true;
                }
            }
            offset++;
            if (offset == param_length) {
                offset = 0;
                state = SPDP_READER_STATE_PARAM_HDR;
            }
            break;
        }
    }

    if (end) {
        state = SPDP_READER_STATE_SBM_DATA_HDR;
        offset = 0;
    }

    out_data.ip_addr[0] = ip_addr[0];
    out_data.ip_addr[1] = ip_addr[1];
    out_data.ip_addr[2] = ip_addr[2];
    out_data.ip_addr[3] = ip_addr[3];
    out_data.udp_port[0] = udp_port[0];
    out_data.udp_port[1] = udp_port[1];
    out.write(out_data);
}
