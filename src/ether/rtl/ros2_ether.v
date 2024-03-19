// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

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
    input  wire [15:0] ros2_rx_udp_port,
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

    input  wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data,
    input  wire [7:0] ros2_pub_app_data_len,
    input  wire ros2_pub_app_data_req,
    input  wire ros2_pub_app_data_rel,
    output wire ros2_pub_app_data_grant,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_addr,
    output wire ros2_sub_app_data_ce,
    output wire ros2_sub_app_data_we,
    output wire [7:0] ros2_sub_app_data_wdata,
    output wire [7:0] ros2_sub_app_data_len,
    input  wire ros2_sub_app_data_req,
    input  wire ros2_sub_app_data_rel,
    output wire ros2_sub_app_data_grant,
    output wire ros2_sub_app_data_recv,

    input  wire udp_rxbuf_rel,
    output wire udp_rxbuf_grant,
    output wire [`UDP_RXBUF_AWIDTH-1:0] udp_rxbuf_addr,
    output wire udp_rxbuf_ce,
    output wire udp_rxbuf_we,
    output wire [31:0] udp_rxbuf_wdata,

    input  wire udp_txbuf_rel,
    output wire udp_txbuf_grant,
    output wire [`UDP_TXBUF_AWIDTH-1:0] udp_txbuf_addr,
    output wire udp_txbuf_ce,
    input  wire [31:0] udp_txbuf_rdata,

    output wire [`PAYLOADSMEM_AWIDTH-1:0] ip_payloadsmem_addr,
    output wire ip_payloadsmem_ce,
    output wire ip_payloadsmem_we,
    output wire [7:0] ip_payloadsmem_wdata,
    input  wire [7:0] ip_payloadsmem_rdata,

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

ros2rapper
ros2rapper (
    .clk(clk),
    .rst_n(rst_n),

    .en(ether_en),
    .ros2pub_en(ros2pub_en),
    .ros2sub_en(ros2sub_en),

    .rx_fifo_dout(rx_fifo_dout),
    .rx_fifo_empty(rx_fifo_empty),
    .rx_fifo_rd_en(rx_fifo_rd_en),

    .tx_fifo_din(tx_fifo_din),
    .tx_fifo_full(tx_fifo_full),
    .tx_fifo_wr_en(tx_fifo_wr_en),

    .ip_addr(ip_addr),
    .subnet_mask(subnet_mask),

    .ros2_node_name(ros2_node_name),
    .ros2_node_name_len(ros2_node_name_len),
    .ros2_node_udp_port(ros2_node_udp_port),
    .ros2_rx_udp_port(ros2_rx_udp_port),
    .ros2_port_num_seed(ros2_port_num_seed),
    .ros2_tx_period(ros2_tx_period),
    .ros2_fragment_expiration(ros2_fragment_expiration),
    .ros2_guid_prefix(ros2_guid_prefix),
    .ros2_ignore_ip_checksum(1'b0),

    .ros2_pub_topic_name(ros2_pub_topic_name),
    .ros2_pub_topic_name_len(ros2_pub_topic_name_len),
    .ros2_pub_topic_type_name(ros2_pub_topic_type_name),
    .ros2_pub_topic_type_name_len(ros2_pub_topic_type_name_len),

    .ros2_sub_topic_name(ros2_sub_topic_name),
    .ros2_sub_topic_name_len(ros2_sub_topic_name_len),
    .ros2_sub_topic_type_name(ros2_sub_topic_type_name),
    .ros2_sub_topic_type_name_len(ros2_sub_topic_type_name_len),

    .ros2_pub_app_data(ros2_pub_app_data),
    .ros2_pub_app_data_len(ros2_pub_app_data_len),
    .ros2_pub_app_data_req(ros2_pub_app_data_req),
    .ros2_pub_app_data_rel(ros2_pub_app_data_rel),
    .ros2_pub_app_data_grant(ros2_pub_app_data_grant),

    .ros2_sub_app_data_addr(ros2_sub_app_data_addr),
    .ros2_sub_app_data_ce(ros2_sub_app_data_ce),
    .ros2_sub_app_data_we(ros2_sub_app_data_we),
    .ros2_sub_app_data_wdata(ros2_sub_app_data_wdata),
    .ros2_sub_app_data_len(ros2_sub_app_data_len),
    .ros2_sub_app_data_req(ros2_sub_app_data_req),
    .ros2_sub_app_data_rel(ros2_sub_app_data_rel),
    .ros2_sub_app_data_grant(ros2_sub_app_data_grant),
    .ros2_sub_app_data_recv(ros2_sub_app_data_recv),

    .udp_rxbuf_rel(udp_rxbuf_rel),
    .udp_rxbuf_grant(udp_rxbuf_grant),
    .udp_rxbuf_addr(udp_rxbuf_addr),
    .udp_rxbuf_ce(udp_rxbuf_ce),
    .udp_rxbuf_we(udp_rxbuf_we),
    .udp_rxbuf_wdata(udp_rxbuf_wdata),

    .udp_txbuf_rel(udp_txbuf_rel),
    .udp_txbuf_grant(udp_txbuf_grant),
    .udp_txbuf_addr(udp_txbuf_addr),
    .udp_txbuf_ce(udp_txbuf_ce),
    .udp_txbuf_rdata(udp_txbuf_rdata),

    .ip_payloadsmem_addr(ip_payloadsmem_addr),
    .ip_payloadsmem_ce(ip_payloadsmem_ce),
    .ip_payloadsmem_we(ip_payloadsmem_we),
    .ip_payloadsmem_wdata(ip_payloadsmem_wdata),
    .ip_payloadsmem_rdata(ip_payloadsmem_rdata)
);

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
