`include "ip.vh"
`default_nettype none

module ros2_eth_tx_adapter (
  input  wire        i_clk,
  input  wire        i_rst_n,
  input  wire        i_enable,
  input  wire [7:0]  i_din_data,
  input  wire        i_din_empty_n,
  output wire        o_din_rd_en,
  output wire        o_tx_hdr_valid,
  input  wire        i_tx_hdr_ready,
  output wire [5:0]  o_tx_ip_dscp,
  output wire [1:0]  o_tx_ip_ecn,
  output wire [15:0] o_tx_ip_length,
  output wire [7:0]  o_tx_ip_ttl,
  output wire [7:0]  o_tx_ip_protocol,
  output wire [31:0] o_tx_ip_source_ip,
  output wire [31:0] o_tx_ip_dest_ip,
  output wire        o_tx_payload_tvalid,
  input  wire        i_tx_payload_tready,
  output wire [7:0]  o_tx_payload_tdata,
  output wire        o_tx_payload_tlast,
  output wire        o_tx_payload_tkeep,
  output wire        o_tx_payload_tstrb
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

  localparam STATE_TX_READ_HDR = 0;
  localparam STATE_TX_HDR      = 1;
  localparam STATE_TX_PAYLOAD  = 2;

  reg [1:0] state;
  reg [15:0] offset;
  reg [15:0] len;
  reg [15:0] counter;

  reg [5:0]  iphdr_dscp;
  reg [1:0]  iphdr_ecn;
  reg [15:0] iphdr_length;
  reg [7:0]  iphdr_ttl;
  reg [7:0]  iphdr_protocol;
  reg [31:0] iphdr_source_ip;
  reg [31:0] iphdr_dest_ip;

  assign o_din_rd_en = (state == STATE_TX_READ_HDR) | (state == STATE_TX_PAYLOAD & i_tx_payload_tready);
  assign o_tx_hdr_valid = (state == STATE_TX_HDR);

  assign o_tx_ip_dscp = iphdr_dscp;
  assign o_tx_ip_ecn = iphdr_ecn;
  assign o_tx_ip_length = iphdr_length;
  assign o_tx_ip_ttl = iphdr_ttl;
  assign o_tx_ip_protocol = iphdr_protocol;
  assign o_tx_ip_source_ip = iphdr_source_ip;
  assign o_tx_ip_dest_ip = iphdr_dest_ip;

  assign o_tx_payload_tvalid = (state == STATE_TX_PAYLOAD & i_din_empty_n);
  assign o_tx_payload_tdata = i_din_data;
  assign o_tx_payload_tlast = (counter + 1 == len);
  assign o_tx_payload_tkeep = 0;
  assign o_tx_payload_tstrb = 0;

  always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
      state <= STATE_TX_READ_HDR;
      offset <= 0;
      len <= 0;
      counter <= 0;
      iphdr_dscp <= 0;
      iphdr_ecn <= 0;
      iphdr_length <= 0;
      iphdr_ttl <= 0;
      iphdr_protocol <= 0;
      iphdr_source_ip <= 0;
      iphdr_dest_ip <= 0;
    end else begin
      if (!i_enable) begin
        state <= STATE_TX_READ_HDR;
        offset <= 0;
      end else begin
        case (state)
        STATE_TX_READ_HDR: begin
          if (i_din_empty_n) begin
            case (offset)
            IP_HDR_OFFSET_TOS: begin
              iphdr_dscp <= i_din_data[7:2];
              iphdr_ecn <= i_din_data[1:0];
            end
            IP_HDR_OFFSET_TOT_LEN:
              iphdr_length[15:8] <= i_din_data;
            IP_HDR_OFFSET_TOT_LEN + 1:
              iphdr_length[7:0] <= i_din_data;
            IP_HDR_OFFSET_TTL:
              iphdr_ttl <= i_din_data;
            IP_HDR_OFFSET_PROTOCOL:
              iphdr_protocol <= i_din_data;
            IP_HDR_OFFSET_SADDR:
              iphdr_source_ip[31:24] <= i_din_data;
            IP_HDR_OFFSET_SADDR + 1:
              iphdr_source_ip[23:16] <= i_din_data;
            IP_HDR_OFFSET_SADDR + 2:
              iphdr_source_ip[15:8] <= i_din_data;
            IP_HDR_OFFSET_SADDR + 3:
              iphdr_source_ip[7:0] <= i_din_data;
            IP_HDR_OFFSET_DADDR:
              iphdr_dest_ip[31:24] <= i_din_data;
            IP_HDR_OFFSET_DADDR + 1:
              iphdr_dest_ip[23:16] <= i_din_data;
            IP_HDR_OFFSET_DADDR + 2:
              iphdr_dest_ip[15:8] <= i_din_data;
            IP_HDR_OFFSET_DADDR + 3:
              iphdr_dest_ip[7:0] <= i_din_data;
            endcase
            if (offset + 1 == IP_HDR_SIZE)
              state <= STATE_TX_HDR;
            else
              offset <= offset + 1;
          end
        end
        STATE_TX_HDR: begin
          if (i_tx_hdr_ready) begin
            state <= (iphdr_length == IP_HDR_SIZE) ? STATE_TX_READ_HDR : STATE_TX_PAYLOAD;
            counter <= 0;
            len <= iphdr_length - IP_HDR_SIZE;
            offset <= 0;
          end
        end
        STATE_TX_PAYLOAD: begin
          if (i_din_empty_n & i_tx_payload_tready) begin
            counter <= counter + 1;
            if (o_tx_payload_tlast) begin
              state <= STATE_TX_READ_HDR;
              offset <= 0;
            end
          end
        end
        default: begin
          // unreachable
          state <= STATE_TX_READ_HDR;
          offset <= 0;
        end
        endcase
      end
    end
  end

endmodule

`default_nettype wire
