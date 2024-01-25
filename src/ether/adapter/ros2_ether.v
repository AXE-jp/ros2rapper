`resetall
`default_nettype none

`include "ros2_config.vh"
`include "ros2_ether_config.vh"

module ros2_ether (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       ether_en,
    input  wire       ros2pub_en,
    input  wire       ros2sub_en,

    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_dv,
    input  wire       phy_rx_er,
    input  wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_en,
    output wire       phy_rst_n,

    input  wire [47:0] mac_addr,
    input  wire [31:0] ip_addr,
    input  wire [31:0] gateway_ip_addr,
    input  wire [31:0] subnet_mask,

    input  wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name,
    input  wire [7:0] ros2_node_name_len,
    input  wire [15:0] ros2_node_udp_port,
    input  wire [15:0] ros2_cpu_udp_port,
    input  wire [15:0] ros2_port_num_seed,
    input  wire [31:0] ros2_tx_period,
    input  wire [31:0] ros2_fragment_expiration,
    input  wire [95:0] ros2_guid_prefix,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name,
    input  wire [7:0] ros2_pub_topic_name_len,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name,
    input  wire [7:0] ros2_pub_topic_type_name_len,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name,
    input  wire [7:0] ros2_sub_topic_name_len,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name,
    input  wire [7:0] ros2_sub_topic_type_name_len,

    input  wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_app_data,
    input  wire [7:0] ros2_app_data_len,
    input  wire ros2_app_data_cpu_req,
    input  wire ros2_app_data_cpu_rel,
    output wire ros2_app_data_cpu_grant,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_app_rx_data_addr,
    output wire ros2_app_rx_data_ce,
    output wire ros2_app_rx_data_we,
    output wire [7:0] ros2_app_rx_data_wdata,
    output wire [7:0] ros2_app_rx_data_len,
    input  wire ros2_app_rx_data_cpu_req,
    input  wire ros2_app_rx_data_cpu_rel,
    output wire ros2_app_rx_data_cpu_grant,

    input  wire udp_rxbuf_cpu_rel,
    output wire udp_rxbuf_cpu_grant,
    output wire [`UDP_RXBUF_AWIDTH-1:0] udp_rxbuf_addr,
    output wire udp_rxbuf_ce,
    output wire udp_rxbuf_we,
    output wire [31:0] udp_rxbuf_wdata,

    input  wire udp_txbuf_cpu_rel,
    output wire udp_txbuf_cpu_grant,
    output wire [`UDP_TXBUF_AWIDTH-1:0] udp_txbuf_addr,
    output wire udp_txbuf_ce,
    input  wire [31:0] udp_txbuf_rdata,

    output wire [`PAYLOADSMEM_AWIDTH-1:0] payloadsmem_addr,
    output wire payloadsmem_ce,
    output wire payloadsmem_we,
    output wire [7:0] payloadsmem_wdata,
    input  wire [7:0] payloadsmem_rdata,

    input  wire [5:0] arp_req_retry_count,
    input  wire [35:0] arp_req_retry_interval,
    input  wire [35:0] arp_req_timeout
);

wire tx_ip_hdr_valid;
wire tx_ip_hdr_ready;
wire [5:0] tx_ip_dscp;
wire [1:0] tx_ip_ecn;
wire [15:0] tx_ip_length;
wire [7:0] tx_ip_ttl;
wire [7:0] tx_ip_protocol;
wire [31:0] tx_ip_source_ip;
wire [31:0] tx_ip_dest_ip;
wire [7:0] tx_ip_payload_axis_tdata;
wire tx_ip_payload_axis_tvalid;
wire tx_ip_payload_axis_tready;
wire tx_ip_payload_axis_tlast;

wire rx_ip_hdr_valid;
wire rx_ip_hdr_ready;
wire [3:0] rx_ip_version;
wire [3:0] rx_ip_ihl;
wire [5:0] rx_ip_dscp;
wire [1:0] rx_ip_ecn;
wire [15:0] rx_ip_length;
wire [15:0] rx_ip_identification;
wire [2:0] rx_ip_flags;
wire [12:0] rx_ip_fragment_offset;
wire [7:0] rx_ip_ttl;
wire [7:0] rx_ip_protocol;
wire [15:0] rx_ip_header_checksum;
wire [31:0] rx_ip_source_ip;
wire [31:0] rx_ip_dest_ip;
wire [7:0] rx_ip_payload_axis_tdata;
wire rx_ip_payload_axis_tvalid;
wire rx_ip_payload_axis_tready;
wire rx_ip_payload_axis_tlast;

verilog_ethernet verilog_ethernet_inst (
    .clk(clk),
    .rst_n(rst_n),
    .enable(ether_en),

    .phy_rx_clk(phy_rx_clk),
    .phy_rxd(phy_rxd),
    .phy_rx_dv(phy_rx_dv),
    .phy_rx_er(phy_rx_er),
    .phy_tx_clk(phy_tx_clk),
    .phy_txd(phy_txd),
    .phy_tx_en(phy_tx_en),
    .phy_reset_n(phy_rst_n),

    .tx_ip_hdr_valid(tx_ip_hdr_valid),
    .tx_ip_hdr_ready(tx_ip_hdr_ready),
    .tx_ip_dscp(tx_ip_dscp),
    .tx_ip_ecn(tx_ip_ecn),
    .tx_ip_length(tx_ip_length),
    .tx_ip_ttl(tx_ip_ttl),
    .tx_ip_protocol(tx_ip_protocol),
    .tx_ip_source_ip(tx_ip_source_ip),
    .tx_ip_dest_ip(tx_ip_dest_ip),
    .tx_ip_payload_axis_tdata(tx_ip_payload_axis_tdata),
    .tx_ip_payload_axis_tvalid(tx_ip_payload_axis_tvalid),
    .tx_ip_payload_axis_tready(tx_ip_payload_axis_tready),
    .tx_ip_payload_axis_tlast(tx_ip_payload_axis_tlast),
    .tx_ip_payload_axis_tuser(1'b0),

    .rx_ip_hdr_valid(rx_ip_hdr_valid),
    .rx_ip_hdr_ready(rx_ip_hdr_ready),
    .rx_ip_version(rx_ip_version),
    .rx_ip_ihl(rx_ip_ihl),
    .rx_ip_dscp(rx_ip_dscp),
    .rx_ip_ecn(rx_ip_ecn),
    .rx_ip_length(rx_ip_length),
    .rx_ip_identification(rx_ip_identification),
    .rx_ip_flags(rx_ip_flags),
    .rx_ip_fragment_offset(rx_ip_fragment_offset),
    .rx_ip_ttl(rx_ip_ttl),
    .rx_ip_protocol(rx_ip_protocol),
    .rx_ip_header_checksum(rx_ip_header_checksum),
    .rx_ip_source_ip(rx_ip_source_ip),
    .rx_ip_dest_ip(rx_ip_dest_ip),
    .rx_ip_payload_axis_tdata(rx_ip_payload_axis_tdata),
    .rx_ip_payload_axis_tvalid(rx_ip_payload_axis_tvalid),
    .rx_ip_payload_axis_tready(rx_ip_payload_axis_tready),
    .rx_ip_payload_axis_tlast(rx_ip_payload_axis_tlast),
    .rx_ip_payload_axis_tuser(),

    .local_mac({48{ether_en}} & {mac_addr[7:0], mac_addr[15:8], mac_addr[23:16], mac_addr[31:24], mac_addr[39:32], mac_addr[47:40]}),
    .local_ip({32{ether_en}} & {ip_addr[7:0], ip_addr[15:8], ip_addr[23:16], ip_addr[31:24]}),
    .gateway_ip({32{ether_en}} & {gateway_ip_addr[7:0], gateway_ip_addr[15:8], gateway_ip_addr[23:16], gateway_ip_addr[31:24]}),
    .subnet_mask({32{ether_en}} & {subnet_mask[7:0], subnet_mask[15:8], subnet_mask[23:16], subnet_mask[31:24]}),

    .arp_req_retry_count(arp_req_retry_count),
    .arp_req_retry_interval(arp_req_retry_interval),
    .arp_req_timeout(arp_req_timeout)
);

wire tx_fifo_wr_en;
wire [7:0] tx_fifo_din;
wire tx_fifo_full;
wire tx_fifo_rd_en;
wire [7:0] tx_fifo_dout;
wire tx_fifo_empty;

queue #(
    .DATA_WIDTH(8),
    .DEPTH(`EXT_TX_FIFO_DEPTH)
)
tx_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .wr_en(tx_fifo_wr_en),
    .din(tx_fifo_din),
    .full(tx_fifo_full),
    .almost_full(),
    .rd_en(tx_fifo_rd_en),
    .dout(tx_fifo_dout),
    .empty(tx_fifo_empty)
);

wire rx_fifo_wr_en;
wire [7:0] rx_fifo_din;
wire rx_fifo_full;
wire rx_fifo_rd_en;
wire [7:0] rx_fifo_dout;
wire rx_fifo_empty;

queue #(
    .DATA_WIDTH(8),
    .DEPTH(`EXT_RX_FIFO_DEPTH)
)
rx_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .wr_en(rx_fifo_wr_en),
    .din(rx_fifo_din),
    .full(rx_fifo_full),
    .almost_full(),
    .rd_en(rx_fifo_rd_en),
    .dout(rx_fifo_dout),
    .empty(rx_fifo_empty)
);

// arbiter for sharing app_data between CPU and ROS2rapper IP
localparam [1:0]
    APP_DATA_GRANT_NONE = 2'b00,
    APP_DATA_GRANT_IP   = 2'b01,
    APP_DATA_GRANT_CPU  = 2'b10;

reg [1:0] r_ros2_app_data_grant;
wire ros2_app_data_ip_req, ros2_app_data_ip_rel, ros2_app_data_ip_grant;
assign ros2_app_data_ip_grant = ether_en & r_ros2_app_data_grant[0];
assign ros2_app_data_cpu_grant = ether_en & r_ros2_app_data_grant[1];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
    end else begin
        case (r_ros2_app_data_grant)
            APP_DATA_GRANT_NONE: begin
                case ({ros2_app_data_ip_req, ros2_app_data_cpu_req})
                    2'b00: r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
                    2'b01: r_ros2_app_data_grant <= APP_DATA_GRANT_CPU;
                    2'b10: r_ros2_app_data_grant <= APP_DATA_GRANT_IP;
                    2'b11: r_ros2_app_data_grant <= APP_DATA_GRANT_IP;
                endcase
            end
            APP_DATA_GRANT_IP:
                if (ros2_app_data_ip_rel) r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
            APP_DATA_GRANT_CPU:
                if (ros2_app_data_cpu_rel) r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
            default:
                r_ros2_app_data_grant <= APP_DATA_GRANT_NONE;
        endcase
    end
end

// arbiter for sharing app_rx_data buffer between CPU and ROS2rapper IP
localparam APP_RX_DATA_GRANT_IP   = 1'b0;
localparam APP_RX_DATA_GRANT_CPU  = 1'b1;

reg r_ros2_app_rx_data_grant;
wire ros2_app_rx_data_ip_grant;
assign ros2_app_rx_data_ip_grant = ros2sub_en & (~r_ros2_app_rx_data_grant);
assign ros2_app_rx_data_cpu_grant = ros2sub_en & r_ros2_app_rx_data_grant;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_ros2_app_rx_data_grant <= APP_RX_DATA_GRANT_IP;
    end else begin
        case (r_ros2_app_rx_data_grant)
            APP_RX_DATA_GRANT_IP:
                if (ros2_app_rx_data_cpu_req) r_ros2_app_rx_data_grant <= APP_RX_DATA_GRANT_CPU;
            APP_RX_DATA_GRANT_CPU:
                if (ros2_app_rx_data_cpu_rel) r_ros2_app_rx_data_grant <= APP_RX_DATA_GRANT_IP;
        endcase
    end
end

// arbiter for sharing UDP RX buffer between CPU and ROS2rapper IP
localparam UDP_RXBUF_GRANT_IP   = 1'b0;
localparam UDP_RXBUF_GRANT_CPU  = 1'b1;

reg r_udp_rxbuf_grant;
wire udp_rxbuf_ip_rel, udp_rxbuf_ip_grant;
assign udp_rxbuf_ip_grant = ether_en & (~r_udp_rxbuf_grant);
assign udp_rxbuf_cpu_grant = ether_en & r_udp_rxbuf_grant;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_IP;
    end else begin
        case (r_udp_rxbuf_grant)
            UDP_RXBUF_GRANT_IP:
                if (udp_rxbuf_ip_rel) r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_CPU;
            UDP_RXBUF_GRANT_CPU:
                if (udp_rxbuf_cpu_rel) r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_IP;
        endcase
    end
end

// arbiter for sharing UDP TX buffer between CPU and ROS2rapper IP
localparam UDP_TXBUF_GRANT_IP   = 1'b0;
localparam UDP_TXBUF_GRANT_CPU  = 1'b1;

reg r_udp_txbuf_grant;
wire udp_txbuf_ip_rel, udp_txbuf_ip_grant;
assign udp_txbuf_ip_grant = ether_en & (~r_udp_txbuf_grant);
assign udp_txbuf_cpu_grant = ether_en & r_udp_txbuf_grant;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_udp_txbuf_grant <= UDP_TXBUF_GRANT_CPU;
    end else begin
        case (r_udp_txbuf_grant)
            UDP_TXBUF_GRANT_IP:
                if (udp_txbuf_ip_rel) r_udp_txbuf_grant <= UDP_TXBUF_GRANT_CPU;
            UDP_TXBUF_GRANT_CPU:
                if (udp_txbuf_cpu_rel) r_udp_txbuf_grant <= UDP_TXBUF_GRANT_IP;
        endcase
    end
end

`ifdef ROS2RAPPER_HLS_VITIS
ros2
ros2 (
    .ap_clk(clk),
    .ap_rst_n(rst_n),
    .pub_enable(ros2pub_en),
    .sub_enable(ros2sub_en),
    .in_r_dout(rx_fifo_dout),
    .in_r_empty_n(~rx_fifo_empty),
    .in_r_read(rx_fifo_rd_en),
    .out_r_din(tx_fifo_din),
    .out_r_full_n(~tx_fifo_full),
    .out_r_write(tx_fifo_wr_en),
    .udp_rxbuf_address0(udp_rxbuf_addr),
    .udp_rxbuf_ce0(udp_rxbuf_ce),
    .udp_rxbuf_we0(udp_rxbuf_we),
    .udp_rxbuf_d0(udp_rxbuf_wdata),
    .udp_txbuf_ce0(udp_txbuf_ce),
    .udp_txbuf_address0(udp_txbuf_addr),
    .udp_txbuf_q0(udp_txbuf_rdata),
    .ip_payloads_address0(payloadsmem_addr),
    .ip_payloads_ce0(payloadsmem_ce),
    .ip_payloads_we0(payloadsmem_we),
    .ip_payloads_d0(payloadsmem_wdata),
    .ip_payloads_q0(payloadsmem_rdata),
    .conf_ip_addr(ip_addr),
    .conf_node_name(ros2_node_name),
    .conf_node_name_len(ros2_node_name_len),
    .conf_node_udp_port({ros2_node_udp_port[7:0], ros2_node_udp_port[15:8]}),
    .conf_cpu_udp_port({ros2_cpu_udp_port[7:0], ros2_cpu_udp_port[15:8]}),
    .conf_port_num_seed(ros2_port_num_seed),
    .conf_tx_period(ros2_tx_period),
    .conf_fragment_expiration(ros2_fragment_expiration),
    .conf_guid_prefix(ros2_guid_prefix),
    .conf_pub_topic_name(ros2_pub_topic_name),
    .conf_pub_topic_name_len(ros2_pub_topic_name_len),
    .conf_pub_topic_type_name(ros2_pub_topic_type_name),
    .conf_pub_topic_type_name_len(ros2_pub_topic_type_name_len),
    .conf_sub_topic_name(ros2_sub_topic_name),
    .conf_sub_topic_name_len(ros2_sub_topic_name_len),
    .conf_sub_topic_type_name(ros2_sub_topic_type_name),
    .conf_sub_topic_type_name_len(ros2_sub_topic_type_name_len),
    .app_data_dout(ros2_app_data),
    .app_data_empty_n(1'b1),
    .app_data_read(),
    .app_data_len_dout(ros2_app_data_len),
    .app_data_len_empty_n(1'b1),
    .app_data_len_read(),
    .app_rx_data_rel_ap_vld(),
    .app_rx_data_rel(),
    .app_rx_data_grant({7'b0, ros2_app_rx_data_ip_grant}),
    .app_rx_data_address0(ros2_app_rx_data_addr),
    .app_rx_data_ce0(ros2_app_rx_data_ce),
    .app_rx_data_we0(ros2_app_rx_data_we),
    .app_rx_data_d0(ros2_app_rx_data_wdata),
    .app_rx_data_len(ros2_app_rx_data_len),
    .app_data_req_ap_vld(ros2_app_data_ip_req),
    .app_data_req(),
    .app_data_rel_ap_vld(ros2_app_data_ip_rel),
    .app_data_rel(),
    .app_data_grant({7'b0, ros2_app_data_ip_grant}),
    .udp_rxbuf_rel_ap_vld(udp_rxbuf_ip_rel),
    .udp_rxbuf_rel(),
    .udp_rxbuf_grant({7'b0, udp_rxbuf_ip_grant}),
    .udp_txbuf_rel_ap_vld(udp_txbuf_ip_rel),
    .udp_txbuf_rel(),
    .udp_txbuf_grant({7'b0, udp_txbuf_ip_grant})
);
`elsif ROS2RAPPER_HLS_CWB
ros2
ros2 (
  .clk(clk),
  .rst_n(rst_n),
  .pub_enable(ros2pub_en),
  .sub_enable(ros2sub_en),
  .in_dout(rx_fifo_dout),
  .in_empty(rx_fifo_empty),
  .in_rreq(rx_fifo_rd_en),
  .out_din(tx_fifo_din),
  .out_full(tx_fifo_full),
  .out_wreq(tx_fifo_wr_en),
  .udp_rxbuf_CS1(udp_rxbuf_ce),
  .udp_rxbuf_AD1(udp_rxbuf_addr),
  .udp_rxbuf_WE1(udp_rxbuf_we),
  .udp_rxbuf_WD1(udp_rxbuf_wdata),
  .udp_txbuf_CS1(udp_txbuf_ce),
  .udp_txbuf_AD1(udp_txbuf_addr),
  .udp_txbuf_RD1(udp_txbuf_rdata),
  .ip_payloads_CS1(payloadsmem_ce),
  .ip_payloads_AD1(payloadsmem_addr),
  .ip_payloads_WE1(payloadsmem_we),
  .ip_payloads_WD1(payloadsmem_wdata),
  .ip_payloads_RD1(payloadsmem_rdata),
  .conf_ip_addr_0(ip_addr[7:0]), .conf_ip_addr_1(ip_addr[15:8]),
  .conf_ip_addr_2(ip_addr[23:16]), .conf_ip_addr_3(ip_addr[31:24]),
  .conf_node_name_00(ros2_node_name[7:0]), .conf_node_name_01(ros2_node_name[15:8]),
  .conf_node_name_02(ros2_node_name[23:16]), .conf_node_name_03(ros2_node_name[31:24]),
  .conf_node_name_04(ros2_node_name[39:32]), .conf_node_name_05(ros2_node_name[47:40]),
  .conf_node_name_06(ros2_node_name[55:48]), .conf_node_name_07(ros2_node_name[63:56]),
  .conf_node_name_08(ros2_node_name[71:64]), .conf_node_name_09(ros2_node_name[79:72]),
  .conf_node_name_10(ros2_node_name[87:80]), .conf_node_name_11(ros2_node_name[95:88]),
  .conf_node_name_12(ros2_node_name[103:96]), .conf_node_name_13(ros2_node_name[111:104]),
  .conf_node_name_14(ros2_node_name[119:112]), .conf_node_name_15(ros2_node_name[127:120]),
  .conf_node_name_16(ros2_node_name[135:128]), .conf_node_name_17(ros2_node_name[143:136]),
  .conf_node_name_18(ros2_node_name[151:144]), .conf_node_name_19(ros2_node_name[159:152]),
  .conf_node_name_20(ros2_node_name[167:160]), .conf_node_name_21(ros2_node_name[175:168]),
  .conf_node_name_22(ros2_node_name[183:176]), .conf_node_name_23(ros2_node_name[191:184]),
  .conf_node_name_24(ros2_node_name[199:192]), .conf_node_name_25(ros2_node_name[207:200]),
  .conf_node_name_26(ros2_node_name[215:208]), .conf_node_name_27(ros2_node_name[223:216]),
  .conf_node_name_28(ros2_node_name[231:224]), .conf_node_name_29(ros2_node_name[239:232]),
  .conf_node_name_30(ros2_node_name[247:240]), .conf_node_name_31(ros2_node_name[255:248]),
  .conf_node_name_len(ros2_node_name_len),
  .conf_node_udp_port_1(ros2_node_udp_port[7:0]), .conf_node_udp_port_0(ros2_node_udp_port[15:8]),
  .conf_cpu_udp_port_1(ros2_cpu_udp_port[7:0]), .conf_cpu_udp_port_0(ros2_cpu_udp_port[15:8]),
  .conf_port_num_seed(ros2_port_num_seed),
  .conf_tx_period(ros2_tx_period),
  .conf_fragment_expiration(ros2_fragment_expiration),
  .conf_guid_prefix_00(ros2_guid_prefix[7:0]), .conf_guid_prefix_01(ros2_guid_prefix[15:8]),
  .conf_guid_prefix_02(ros2_guid_prefix[23:16]), .conf_guid_prefix_03(ros2_guid_prefix[31:24]),
  .conf_guid_prefix_04(ros2_guid_prefix[39:32]), .conf_guid_prefix_05(ros2_guid_prefix[47:40]),
  .conf_guid_prefix_06(ros2_guid_prefix[55:48]), .conf_guid_prefix_07(ros2_guid_prefix[63:56]),
  .conf_guid_prefix_08(ros2_guid_prefix[71:64]), .conf_guid_prefix_09(ros2_guid_prefix[79:72]),
  .conf_guid_prefix_10(ros2_guid_prefix[87:80]), .conf_guid_prefix_11(ros2_guid_prefix[95:88]),
  .conf_topic_name_00(ros2_topic_name[7:0]), .conf_topic_name_01(ros2_topic_name[15:8]),
  .conf_topic_name_02(ros2_topic_name[23:16]), .conf_topic_name_03(ros2_topic_name[31:24]),
  .conf_topic_name_04(ros2_topic_name[39:32]), .conf_topic_name_05(ros2_topic_name[47:40]),
  .conf_topic_name_06(ros2_topic_name[55:48]), .conf_topic_name_07(ros2_topic_name[63:56]),
  .conf_topic_name_08(ros2_topic_name[71:64]), .conf_topic_name_09(ros2_topic_name[79:72]),
  .conf_topic_name_10(ros2_topic_name[87:80]), .conf_topic_name_11(ros2_topic_name[95:88]),
  .conf_topic_name_12(ros2_topic_name[103:96]), .conf_topic_name_13(ros2_topic_name[111:104]),
  .conf_topic_name_14(ros2_topic_name[119:112]), .conf_topic_name_15(ros2_topic_name[127:120]),
  .conf_topic_name_16(ros2_topic_name[135:128]), .conf_topic_name_17(ros2_topic_name[143:136]),
  .conf_topic_name_18(ros2_topic_name[151:144]), .conf_topic_name_19(ros2_topic_name[159:152]),
  .conf_topic_name_20(ros2_topic_name[167:160]), .conf_topic_name_21(ros2_topic_name[175:168]),
  .conf_topic_name_22(ros2_topic_name[183:176]), .conf_topic_name_23(ros2_topic_name[191:184]),
  .conf_topic_name_24(ros2_topic_name[199:192]), .conf_topic_name_25(ros2_topic_name[207:200]),
  .conf_topic_name_26(ros2_topic_name[215:208]), .conf_topic_name_27(ros2_topic_name[223:216]),
  .conf_topic_name_28(ros2_topic_name[231:224]), .conf_topic_name_29(ros2_topic_name[239:232]),
  .conf_topic_name_30(ros2_topic_name[247:240]), .conf_topic_name_31(ros2_topic_name[255:248]),
  .conf_topic_name_len(ros2_topic_name_len),
  .conf_topic_type_name_00(ros2_topic_type_name[7:0]), .conf_topic_type_name_01(ros2_topic_type_name[15:8]),
  .conf_topic_type_name_02(ros2_topic_type_name[23:16]), .conf_topic_type_name_03(ros2_topic_type_name[31:24]),
  .conf_topic_type_name_04(ros2_topic_type_name[39:32]), .conf_topic_type_name_05(ros2_topic_type_name[47:40]),
  .conf_topic_type_name_06(ros2_topic_type_name[55:48]), .conf_topic_type_name_07(ros2_topic_type_name[63:56]),
  .conf_topic_type_name_08(ros2_topic_type_name[71:64]), .conf_topic_type_name_09(ros2_topic_type_name[79:72]),
  .conf_topic_type_name_10(ros2_topic_type_name[87:80]), .conf_topic_type_name_11(ros2_topic_type_name[95:88]),
  .conf_topic_type_name_12(ros2_topic_type_name[103:96]), .conf_topic_type_name_13(ros2_topic_type_name[111:104]),
  .conf_topic_type_name_14(ros2_topic_type_name[119:112]), .conf_topic_type_name_15(ros2_topic_type_name[127:120]),
  .conf_topic_type_name_16(ros2_topic_type_name[135:128]), .conf_topic_type_name_17(ros2_topic_type_name[143:136]),
  .conf_topic_type_name_18(ros2_topic_type_name[151:144]), .conf_topic_type_name_19(ros2_topic_type_name[159:152]),
  .conf_topic_type_name_20(ros2_topic_type_name[167:160]), .conf_topic_type_name_21(ros2_topic_type_name[175:168]),
  .conf_topic_type_name_22(ros2_topic_type_name[183:176]), .conf_topic_type_name_23(ros2_topic_type_name[191:184]),
  .conf_topic_type_name_24(ros2_topic_type_name[199:192]), .conf_topic_type_name_25(ros2_topic_type_name[207:200]),
  .conf_topic_type_name_26(ros2_topic_type_name[215:208]), .conf_topic_type_name_27(ros2_topic_type_name[223:216]),
  .conf_topic_type_name_28(ros2_topic_type_name[231:224]), .conf_topic_type_name_29(ros2_topic_type_name[239:232]),
  .conf_topic_type_name_30(ros2_topic_type_name[247:240]), .conf_topic_type_name_31(ros2_topic_type_name[255:248]),
  .conf_topic_type_name_32(ros2_topic_type_name[263:256]), .conf_topic_type_name_33(ros2_topic_type_name[271:264]),
  .conf_topic_type_name_34(ros2_topic_type_name[279:272]), .conf_topic_type_name_35(ros2_topic_type_name[287:280]),
  .conf_topic_type_name_36(ros2_topic_type_name[295:288]), .conf_topic_type_name_37(ros2_topic_type_name[303:296]),
  .conf_topic_type_name_38(ros2_topic_type_name[311:304]), .conf_topic_type_name_39(ros2_topic_type_name[319:312]),
  .conf_topic_type_name_40(ros2_topic_type_name[327:320]), .conf_topic_type_name_41(ros2_topic_type_name[335:328]),
  .conf_topic_type_name_42(ros2_topic_type_name[343:336]), .conf_topic_type_name_43(ros2_topic_type_name[351:344]),
  .conf_topic_type_name_44(ros2_topic_type_name[359:352]), .conf_topic_type_name_45(ros2_topic_type_name[367:360]),
  .conf_topic_type_name_46(ros2_topic_type_name[375:368]), .conf_topic_type_name_47(ros2_topic_type_name[383:376]),
  .conf_topic_type_name_48(ros2_topic_type_name[391:384]), .conf_topic_type_name_49(ros2_topic_type_name[399:392]),
  .conf_topic_type_name_50(ros2_topic_type_name[407:400]), .conf_topic_type_name_51(ros2_topic_type_name[415:408]),
  .conf_topic_type_name_52(ros2_topic_type_name[423:416]), .conf_topic_type_name_53(ros2_topic_type_name[431:424]),
  .conf_topic_type_name_54(ros2_topic_type_name[439:432]), .conf_topic_type_name_55(ros2_topic_type_name[447:440]),
  .conf_topic_type_name_56(ros2_topic_type_name[455:448]), .conf_topic_type_name_57(ros2_topic_type_name[463:456]),
  .conf_topic_type_name_58(ros2_topic_type_name[471:464]), .conf_topic_type_name_59(ros2_topic_type_name[479:472]),
  .conf_topic_type_name_60(ros2_topic_type_name[487:480]), .conf_topic_type_name_61(ros2_topic_type_name[495:488]),
  .conf_topic_type_name_62(ros2_topic_type_name[503:496]), .conf_topic_type_name_63(ros2_topic_type_name[511:504]),
  .conf_topic_type_name_len(ros2_topic_type_name_len),
  .app_data_00_rd(ros2_app_data[7:0]),     .app_data_01_rd(ros2_app_data[15:8]),
  .app_data_02_rd(ros2_app_data[23:16]),   .app_data_03_rd(ros2_app_data[31:24]),
  .app_data_04_rd(ros2_app_data[39:32]),   .app_data_05_rd(ros2_app_data[47:40]),
  .app_data_06_rd(ros2_app_data[55:48]),   .app_data_07_rd(ros2_app_data[63:56]),
  .app_data_08_rd(ros2_app_data[71:64]),   .app_data_09_rd(ros2_app_data[79:72]),
  .app_data_10_rd(ros2_app_data[87:80]),   .app_data_11_rd(ros2_app_data[95:88]),
  .app_data_12_rd(ros2_app_data[103:96]),  .app_data_13_rd(ros2_app_data[111:104]),
  .app_data_14_rd(ros2_app_data[119:112]), .app_data_15_rd(ros2_app_data[127:120]),
  .app_data_16_rd(ros2_app_data[135:128]), .app_data_17_rd(ros2_app_data[143:136]),
  .app_data_18_rd(ros2_app_data[151:144]), .app_data_19_rd(ros2_app_data[159:152]),
  .app_data_20_rd(ros2_app_data[167:160]), .app_data_21_rd(ros2_app_data[175:168]),
  .app_data_22_rd(ros2_app_data[183:176]), .app_data_23_rd(ros2_app_data[191:184]),
  .app_data_24_rd(ros2_app_data[199:192]), .app_data_25_rd(ros2_app_data[207:200]),
  .app_data_26_rd(ros2_app_data[215:208]), .app_data_27_rd(ros2_app_data[223:216]),
  .app_data_28_rd(ros2_app_data[231:224]), .app_data_29_rd(ros2_app_data[239:232]),
  .app_data_30_rd(ros2_app_data[247:240]), .app_data_31_rd(ros2_app_data[255:248]),
  .app_data_32_rd(ros2_app_data[263:256]), .app_data_33_rd(ros2_app_data[271:264]),
  .app_data_34_rd(ros2_app_data[279:272]), .app_data_35_rd(ros2_app_data[287:280]),
  .app_data_36_rd(ros2_app_data[295:288]), .app_data_37_rd(ros2_app_data[303:296]),
  .app_data_38_rd(ros2_app_data[311:304]), .app_data_39_rd(ros2_app_data[319:312]),
  .app_data_40_rd(ros2_app_data[327:320]), .app_data_41_rd(ros2_app_data[335:328]),
  .app_data_42_rd(ros2_app_data[343:336]), .app_data_43_rd(ros2_app_data[351:344]),
  .app_data_44_rd(ros2_app_data[359:352]), .app_data_45_rd(ros2_app_data[367:360]),
  .app_data_46_rd(ros2_app_data[375:368]), .app_data_47_rd(ros2_app_data[383:376]),
  .app_data_48_rd(ros2_app_data[391:384]), .app_data_49_rd(ros2_app_data[399:392]),
  .app_data_50_rd(ros2_app_data[407:400]), .app_data_51_rd(ros2_app_data[415:408]),
  .app_data_52_rd(ros2_app_data[423:416]), .app_data_53_rd(ros2_app_data[431:424]),
  .app_data_54_rd(ros2_app_data[439:432]), .app_data_55_rd(ros2_app_data[447:440]),
  .app_data_56_rd(ros2_app_data[455:448]), .app_data_57_rd(ros2_app_data[463:456]),
  .app_data_58_rd(ros2_app_data[471:464]), .app_data_59_rd(ros2_app_data[479:472]),
  .app_data_60_rd(ros2_app_data[487:480]), .app_data_61_rd(ros2_app_data[495:488]),
  .app_data_62_rd(ros2_app_data[503:496]), .app_data_63_rd(ros2_app_data[511:504]),
  .app_data_len_rreq(),
  .app_data_len_empty(1'b0),
  .app_data_len_dout(ros2_app_data_len),
  .app_data_req_we(ros2_app_data_ip_req),
  .app_data_req_wd(),
  .app_data_rel_we(ros2_app_data_ip_rel),
  .app_data_rel_wd(),
  .app_data_grant_rd({7'b0, ros2_app_data_ip_grant}),
  .udp_rxbuf_rel_we(udp_rxbuf_ip_rel),
  .udp_rxbuf_rel_wd(),
  .udp_rxbuf_grant_rd({7'b0, udp_rxbuf_ip_grant}),
  .udp_txbuf_rel_we(udp_txbuf_ip_rel),
  .udp_txbuf_rel_wd(),
  .udp_txbuf_grant_rd({7'b0, udp_txbuf_ip_grant})
);
`endif

ros2_eth_tx_adapter
ros2_eth_tx_adapter (
    .i_clk(clk),
    .i_rst_n(rst_n),
    .i_enable(ether_en),
    .i_din_data(tx_fifo_dout),
    .i_din_empty_n(~tx_fifo_empty),
    .o_din_rd_en(tx_fifo_rd_en),
    .o_tx_hdr_valid(tx_ip_hdr_valid),
    .i_tx_hdr_ready(tx_ip_hdr_ready),
    .o_tx_ip_dest_ip(tx_ip_dest_ip),
    .o_tx_ip_source_ip(tx_ip_source_ip),
    .o_tx_ip_protocol(tx_ip_protocol),
    .o_tx_ip_ttl(tx_ip_ttl),
    .o_tx_ip_length(tx_ip_length),
    .o_tx_ip_ecn(tx_ip_ecn),
    .o_tx_ip_dscp(tx_ip_dscp),
    .o_tx_payload_tvalid(tx_ip_payload_axis_tvalid),
    .i_tx_payload_tready(tx_ip_payload_axis_tready),
    .o_tx_payload_tdata(tx_ip_payload_axis_tdata),
    .o_tx_payload_tlast(tx_ip_payload_axis_tlast),
    .o_tx_payload_tkeep(),
    .o_tx_payload_tstrb()
);

ros2_eth_rx_adapter
ros2_eth_rx_adapter (
    .i_clk(clk),
    .i_rst_n(rst_n),
    .i_enable(ether_en),
    .o_dout_data(rx_fifo_din),
    .i_dout_full_n(~rx_fifo_full),
    .o_dout_wr_en(rx_fifo_wr_en),
    .i_rx_hdr_valid(rx_ip_hdr_valid),
    .o_rx_hdr_ready(rx_ip_hdr_ready),
    .i_rx_ip_dest_ip(rx_ip_dest_ip),
    .i_rx_ip_source_ip(rx_ip_source_ip),
    .i_rx_ip_header_checksum(rx_ip_header_checksum),
    .i_rx_ip_protocol(rx_ip_protocol),
    .i_rx_ip_ttl(rx_ip_ttl),
    .i_rx_ip_fragment_offset(rx_ip_fragment_offset),
    .i_rx_ip_flags(rx_ip_flags),
    .i_rx_ip_identification(rx_ip_identification),
    .i_rx_ip_length(rx_ip_length),
    .i_rx_ip_ecn(rx_ip_ecn),
    .i_rx_ip_dscp(rx_ip_dscp),
    .i_rx_ip_ihl(rx_ip_ihl),
    .i_rx_ip_version(rx_ip_version),
    .i_rx_payload_tvalid(rx_ip_payload_axis_tvalid),
    .o_rx_payload_tready(rx_ip_payload_axis_tready),
    .i_rx_payload_tdata(rx_ip_payload_axis_tdata),
    .i_rx_payload_tlast(rx_ip_payload_axis_tlast),
    .i_rx_payload_tkeep(1'b1),
    .i_rx_payload_tstrb(1'b1)
);

endmodule

`resetall
