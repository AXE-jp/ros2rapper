`include "ip.vh"
`default_nettype none

module ros2_eth_rx_adapter (
  input  wire        i_clk,
  input  wire        i_rst_n,
  input  wire        i_enable,
  output wire [7:0]  o_dout_data,
  input  wire        i_dout_full_n,
  output wire        o_dout_wr_en,
  input  wire        i_rx_hdr_valid,
  output wire        o_rx_hdr_ready,
  input  wire [31:0] i_rx_ip_dest_ip,
  input  wire [31:0] i_rx_ip_source_ip,
  input  wire [15:0] i_rx_ip_header_checksum,
  input  wire [7:0]  i_rx_ip_protocol,
  input  wire [7:0]  i_rx_ip_ttl,
  input  wire [12:0] i_rx_ip_fragment_offset,
  input  wire [2:0]  i_rx_ip_flags,
  input  wire [15:0] i_rx_ip_identification,
  input  wire [15:0] i_rx_ip_length,
  input  wire [1:0]  i_rx_ip_ecn,
  input  wire [5:0]  i_rx_ip_dscp,
  input  wire [3:0]  i_rx_ip_ihl,
  input  wire [3:0]  i_rx_ip_version,
  input  wire        i_rx_payload_tvalid,
  output wire        o_rx_payload_tready,
  input  wire [7:0]  i_rx_payload_tdata,
  input  wire        i_rx_payload_tlast,
  input  wire        i_rx_payload_tkeep,
  input  wire        i_rx_payload_tstrb
);

  localparam IP_HDR_SIZE = 20;

  localparam IP_HDR_OFFSET_VERSION_IHL = 0;  // Version, IHL
  localparam IP_HDR_OFFSET_TOS         = 1;  // Type of Service
  localparam IP_HDR_OFFSET_TOT_LEN     = 2;  // Total Length
  localparam IP_HDR_OFFSET_ID          = 4;  // Identification
  localparam IP_HDR_OFFSET_FLAG_OFF    = 6;  // Flags, Fragment Offset
  localparam IP_HDR_OFFSET_TTL         = 8;  // Time to Live
  localparam IP_HDR_OFFSET_PROTOCOL    = 9;  // Protocol
  localparam IP_HDR_OFFSET_CHECK       = 10; // Header Checksum
  localparam IP_HDR_OFFSET_SADDR       = 12; // Source Address
  localparam IP_HDR_OFFSET_DADDR       = 16; // Destination Address

  localparam STATE_RX_HDR       = 0;
  localparam STATE_RX_WRITE_HDR = 1;
  localparam STATE_RX_PAYLOAD   = 2;

  reg [1:0] state;
  reg [15:0] offset;

  reg [3:0]  iphdr_version;
  reg [3:0]  iphdr_ihl;
  reg [5:0]  iphdr_dscp;
  reg [1:0]  iphdr_ecn;
  reg [15:0] iphdr_length;
  reg [15:0] iphdr_identification;
  reg [2:0]  iphdr_flags;
  reg [12:0] iphdr_fragment_offset;
  reg [7:0]  iphdr_ttl;
  reg [7:0]  iphdr_protocol;
  reg [15:0] iphdr_header_checksum;
  reg [31:0] iphdr_source_ip;
  reg [31:0] iphdr_dest_ip;

  reg [7:0] dout_data;
  always @(*) begin
    if (state == STATE_RX_WRITE_HDR) begin
      case (offset)
      IP_HDR_OFFSET_VERSION_IHL:
        dout_data = {iphdr_version, iphdr_ihl};
      IP_HDR_OFFSET_TOS:
        dout_data = {iphdr_dscp, iphdr_ecn};
      IP_HDR_OFFSET_TOT_LEN:
        dout_data = iphdr_length[15:8];
      IP_HDR_OFFSET_TOT_LEN + 1:
        dout_data = iphdr_length[7:0];
      IP_HDR_OFFSET_ID:
        dout_data = iphdr_identification[15:8];
      IP_HDR_OFFSET_ID + 1:
        dout_data = iphdr_identification[7:0];
      IP_HDR_OFFSET_FLAG_OFF:
        dout_data = {iphdr_flags, iphdr_fragment_offset[12:8]};
      IP_HDR_OFFSET_FLAG_OFF + 1:
        dout_data = iphdr_fragment_offset[7:0];
      IP_HDR_OFFSET_TTL:
        dout_data = iphdr_ttl;
      IP_HDR_OFFSET_PROTOCOL:
        dout_data = iphdr_protocol;
      IP_HDR_OFFSET_CHECK:
        dout_data = iphdr_header_checksum[15:8];
      IP_HDR_OFFSET_CHECK + 1:
        dout_data = iphdr_header_checksum[7:0];
      IP_HDR_OFFSET_SADDR:
        dout_data = iphdr_source_ip[31:24];
      IP_HDR_OFFSET_SADDR + 1:
        dout_data = iphdr_source_ip[23:16];
      IP_HDR_OFFSET_SADDR + 2:
        dout_data = iphdr_source_ip[15:8];
      IP_HDR_OFFSET_SADDR + 3:
        dout_data = iphdr_source_ip[7:0];
      IP_HDR_OFFSET_DADDR:
        dout_data = iphdr_dest_ip[31:24];
      IP_HDR_OFFSET_DADDR + 1:
        dout_data = iphdr_dest_ip[23:16];
      IP_HDR_OFFSET_DADDR + 2:
        dout_data = iphdr_dest_ip[15:8];
      IP_HDR_OFFSET_DADDR + 3:
        dout_data = iphdr_dest_ip[7:0];
      default:
        dout_data = 0;
      endcase
    end else begin
      dout_data = i_rx_payload_tdata;
    end
  end

  assign o_rx_hdr_ready = (state == STATE_RX_HDR);
  assign o_dout_data = dout_data;
  assign o_dout_wr_en = (state == STATE_RX_WRITE_HDR) | (state == STATE_RX_PAYLOAD & i_rx_payload_tvalid);
  assign o_rx_payload_tready = (state == STATE_RX_PAYLOAD & i_dout_full_n);

  always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
      state <= STATE_RX_HDR;
      offset <= 0;
      iphdr_version <= 0;
      iphdr_ihl <= 0;
      iphdr_dscp <= 0;
      iphdr_ecn <= 0;
      iphdr_length <= 0;
      iphdr_identification <= 0;
      iphdr_flags <= 0;
      iphdr_fragment_offset <= 0;
      iphdr_ttl <= 0;
      iphdr_protocol <= 0;
      iphdr_header_checksum <= 0;
      iphdr_source_ip <= 0;
      iphdr_dest_ip <= 0;
    end else begin
      if (!i_enable) begin
        state <= STATE_RX_HDR;
      end else begin
        case (state)
        STATE_RX_HDR: begin
          if (i_rx_hdr_valid) begin
            iphdr_version <= i_rx_ip_version;
            iphdr_ihl <= i_rx_ip_ihl;
            iphdr_dscp <= i_rx_ip_dscp;
            iphdr_ecn <= i_rx_ip_ecn;
            iphdr_length <= i_rx_ip_length;
            iphdr_identification <= i_rx_ip_identification;
            iphdr_flags <= i_rx_ip_flags;
            iphdr_fragment_offset <= i_rx_ip_fragment_offset;
            iphdr_ttl <= i_rx_ip_ttl;
            iphdr_protocol <= i_rx_ip_protocol;
            iphdr_header_checksum <= i_rx_ip_header_checksum;
            iphdr_source_ip <= i_rx_ip_source_ip;
            iphdr_dest_ip <= i_rx_ip_dest_ip;
            offset <= 0;
            state <= STATE_RX_WRITE_HDR;
          end
        end
        STATE_RX_WRITE_HDR: begin
          if (i_dout_full_n) begin
            if (offset + 1 == IP_HDR_SIZE)
              state <= STATE_RX_PAYLOAD;
            else
              offset <= offset + 1;
          end
        end
        STATE_RX_PAYLOAD: begin
          if (i_rx_payload_tvalid & i_dout_full_n & i_rx_payload_tlast)
            state <= STATE_RX_HDR;
        end
        default: begin
          // unreachable
          state <= STATE_RX_HDR;
        end
        endcase
      end
    end
  end

`ifdef FORMAL
    // preparation for using $past
    reg f_past_valid;
    initial f_past_valid = 1'b0;
    always @(posedge i_clk)
        f_past_valid <= 1'b1;

    // reset
    initial assume(~i_rst_n);
    always @(posedge i_clk) begin
        if (!f_past_valid)
            assume(~i_rst_n);
        else begin
            assume(i_rst_n);
        end
    end

    always @(posedge i_clk) begin
        if (f_past_valid) begin
            if ($past(~i_rst_n | ~i_enable)) begin
                state_transition_check_0: assert(state == STATE_RX_HDR);
            end else begin
                if ($past(state == STATE_RX_HDR))
                    state_transition_check_1: assert(state == STATE_RX_HDR | state == STATE_RX_WRITE_HDR);
                else if ($past(state == STATE_RX_WRITE_HDR))
                    state_transition_check_2: assert(state == STATE_RX_WRITE_HDR | state == STATE_RX_PAYLOAD);
                else if ($past(state == STATE_RX_PAYLOAD))
                    state_transition_check_3: assert(state == STATE_RX_PAYLOAD | state == STATE_RX_HDR);

                assert(i_dout_full_n);
            end
        end
    end

    always @(posedge i_clk) begin
    end
`endif

endmodule

`default_nettype wire
