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

  localparam STATE_RX_HDR       = 0;
  localparam STATE_RX_WRITE_HDR = 1;
  localparam STATE_RX_PAYLOAD   = 2;

  reg [1:0] state;
  reg [15:0] offset;

  reg [31:0] iphdr_dest_ip;
  reg [31:0] iphdr_source_ip;
  reg [15:0] iphdr_header_checksum;
  reg [7:0]  iphdr_protocol;
  reg [7:0]  iphdr_ttl;
  reg [12:0] iphdr_fragment_offset;
  reg [2:0]  iphdr_flags;
  reg [15:0] iphdr_identification;
  reg [15:0] iphdr_length;
  reg [1:0]  iphdr_ecn;
  reg [5:0]  iphdr_dscp;
  reg [3:0]  iphdr_ihl;
  reg [3:0]  iphdr_version;

  reg [7:0] dout_data;
  always @(*) begin
    if (state == STATE_RX_WRITE_HDR) begin
      case (offset)
      `IP_HDR_OFFSET_VERSION_IHL:
        dout_data = {iphdr_version, iphdr_ihl};
      `IP_HDR_OFFSET_TOS:
        dout_data = {iphdr_dscp, iphdr_ecn};
      `IP_HDR_OFFSET_TOT_LEN:
        dout_data = iphdr_length[15:8];
      `IP_HDR_OFFSET_TOT_LEN + 1:
        dout_data = iphdr_length[7:0];
      `IP_HDR_OFFSET_ID:
        dout_data = iphdr_identification[15:8];
      `IP_HDR_OFFSET_ID + 1:
        dout_data = iphdr_identification[7:0];
      `IP_HDR_OFFSET_FLAG_OFF:
        dout_data = {iphdr_flags, iphdr_fragment_offset[12:8]};
      `IP_HDR_OFFSET_FLAG_OFF + 1:
        dout_data = iphdr_fragment_offset[7:0];
      `IP_HDR_OFFSET_TTL:
        dout_data = iphdr_ttl;
      `IP_HDR_OFFSET_PROTOCOL:
        dout_data = iphdr_protocol;
      `IP_HDR_OFFSET_CHECK:
        dout_data = iphdr_header_checksum[15:8];
      `IP_HDR_OFFSET_CHECK + 1:
        dout_data = iphdr_header_checksum[7:0];
      `IP_HDR_OFFSET_SADDR:
        dout_data = iphdr_source_ip[31:24];
      `IP_HDR_OFFSET_SADDR + 1:
        dout_data = iphdr_source_ip[23:16];
      `IP_HDR_OFFSET_SADDR + 2:
        dout_data = iphdr_source_ip[15:8];
      `IP_HDR_OFFSET_SADDR + 3:
        dout_data = iphdr_source_ip[7:0];
      `IP_HDR_OFFSET_DADDR:
        dout_data = iphdr_dest_ip[31:24];
      `IP_HDR_OFFSET_DADDR + 1:
        dout_data = iphdr_dest_ip[23:16];
      `IP_HDR_OFFSET_DADDR + 2:
        dout_data = iphdr_dest_ip[15:8];
      `IP_HDR_OFFSET_DADDR + 3:
        dout_data = iphdr_dest_ip[7:0];
      endcase
    end else begin
      dout_data = i_rx_payload_tdata;
    end
  end

  wire o_rx_hdr_ready = (state == STATE_RX_HDR);
  wire o_dout_data = dout_data;
  wire o_dout_valid = (state == STATE_RX_WRITE_HDR) | (state == STATE_RX_PAYLOAD & i_dout_full_n);

  always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
      state <= STATE_RX_HDR;
      offset <= 0;
    end else begin
      if (!enable) begin
        state <= STATE_RX_HDR;
      end else begin
        case (state)
        STATE_RX_HDR: begin
          if (i_rx_hdr_valid) begin
            iphdr_dest_ip <= i_rx_ip_dest_ip;
            iphdr_source_ip <= i_rx_ip_source_ip;
            iphdr_header_checksum <= i_rx_ip_header_checksum;
            iphdr_protocol <= i_rx_ip_protocol;
            iphdr_ttl <= i_rx_ip_ttl;
            iphdr_fragment_offset <= i_rx_ip_fragment_offset;
            iphdr_flags <= i_rx_ip_flags;
            iphdr_identification <= i_rx_ip_identification;
            iphdr_length <= i_rx_ip_length;
            iphdr_ecn <= i_rx_ip_ecn;
            iphdr_dscp <= i_rx_ip_dscp;
            iphdr_ihl <= i_rx_ip_ihl;
            iphdr_version <= i_rx_ip_version;
            offset <= 0;
            state <= STATE_RX_WRITE_HDR;
          end
        end
        STATE_RX_WRITE_HDR: begin
          if (i_dout_full_n) begin
            if (offset + 1 == `IP_HDR_SIZE)
              state <= STATE_RX_PAYLOAD;
            else
              offset <= offset + 1;
          end
        end
        STATE_RX_PAYLOAD: begin
          if (o_dout_valid & i_rx_payload_tlast)
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

endmodule

`default_nettype wire
