// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none

`include "ros2_config.vh"

module ros2rapper (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       en,
    input  wire       ros2pub_en,
    input  wire       ros2sub_en,

    input  wire [7:0] rx_fifo_dout,
    input  wire       rx_fifo_empty,
    output wire       rx_fifo_rd_en,

    output wire [7:0] tx_fifo_din,
    input  wire       tx_fifo_full,
    output wire       tx_fifo_wr_en,

    input  wire [31:0] ip_addr,
    input  wire [31:0] subnet_mask,

    input  wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name,
    input  wire [7:0] ros2_node_name_len,
    input  wire [15:0] ros2_node_udp_port,
    input  wire [15:0] ros2_rx_udp_port,
    input  wire [15:0] ros2_port_num_seed,
    input  wire [31:0] ros2_tx_period,
    input  wire [31:0] ros2_fragment_expiration,
    input  wire [95:0] ros2_guid_prefix,
    input  wire        ros2_ignore_ip_checksum,

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
    output wire [15:0] ros2_sub_app_data_rep_id,
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
    input  wire [7:0] ip_payloadsmem_rdata
);

// arbiter for sharing publisher app_data between user and IP
localparam [1:0]
    PUB_APP_DATA_GRANT_NONE = 2'b00,
    PUB_APP_DATA_GRANT_IP   = 2'b01,
    PUB_APP_DATA_GRANT_USER = 2'b10;

reg [1:0] r_ros2_pub_app_data_grant;
wire ros2_pub_app_data_ip_req, ros2_pub_app_data_ip_rel, ros2_pub_app_data_ip_grant;
assign ros2_pub_app_data_ip_grant = en & r_ros2_pub_app_data_grant[0];
assign ros2_pub_app_data_grant = en & r_ros2_pub_app_data_grant[1];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_ros2_pub_app_data_grant <= PUB_APP_DATA_GRANT_NONE;
    end else begin
        case (r_ros2_pub_app_data_grant)
            PUB_APP_DATA_GRANT_NONE: begin
                case ({ros2_pub_app_data_ip_req, ros2_pub_app_data_req})
                    2'b00: r_ros2_pub_app_data_grant <= PUB_APP_DATA_GRANT_NONE;
                    2'b01: r_ros2_pub_app_data_grant <= PUB_APP_DATA_GRANT_USER;
                    2'b10: r_ros2_pub_app_data_grant <= PUB_APP_DATA_GRANT_IP;
                    2'b11: r_ros2_pub_app_data_grant <= PUB_APP_DATA_GRANT_IP;
                endcase
            end
            PUB_APP_DATA_GRANT_IP:
                if (ros2_pub_app_data_ip_rel) r_ros2_pub_app_data_grant <= PUB_APP_DATA_GRANT_NONE;
            PUB_APP_DATA_GRANT_USER:
                if (ros2_pub_app_data_rel) r_ros2_pub_app_data_grant <= PUB_APP_DATA_GRANT_NONE;
            default:
                r_ros2_pub_app_data_grant <= PUB_APP_DATA_GRANT_NONE;
        endcase
    end
end

// arbiter for sharing subscriber app_data buffer between user and IP
localparam SUB_APP_DATA_GRANT_IP   = 1'b0;
localparam SUB_APP_DATA_GRANT_USER = 1'b1;

reg r_ros2_sub_app_data_grant;
wire ros2_sub_app_data_ip_grant;
assign ros2_sub_app_data_ip_grant = (en & ros2sub_en) & (~r_ros2_sub_app_data_grant);
assign ros2_sub_app_data_grant =(en &  ros2sub_en) & r_ros2_sub_app_data_grant;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_ros2_sub_app_data_grant <= SUB_APP_DATA_GRANT_IP;
    end else begin
        case (r_ros2_sub_app_data_grant)
            SUB_APP_DATA_GRANT_IP:
                if (ros2_sub_app_data_req) r_ros2_sub_app_data_grant <= SUB_APP_DATA_GRANT_USER;
            SUB_APP_DATA_GRANT_USER:
                if (ros2_sub_app_data_rel) r_ros2_sub_app_data_grant <= SUB_APP_DATA_GRANT_IP;
        endcase
    end
end

// arbiter for sharing UDP RX buffer between user and ROS2rapper IP
localparam UDP_RXBUF_GRANT_IP   = 1'b0;
localparam UDP_RXBUF_GRANT_USER = 1'b1;

reg r_udp_rxbuf_grant;
wire udp_rxbuf_ip_rel, udp_rxbuf_ip_grant;
assign udp_rxbuf_ip_grant = en & (~r_udp_rxbuf_grant);
assign udp_rxbuf_grant = en & r_udp_rxbuf_grant;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_IP;
    end else begin
        case (r_udp_rxbuf_grant)
            UDP_RXBUF_GRANT_IP:
                if (udp_rxbuf_ip_rel) r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_USER;
            UDP_RXBUF_GRANT_USER:
                if (udp_rxbuf_rel) r_udp_rxbuf_grant <= UDP_RXBUF_GRANT_IP;
        endcase
    end
end

// arbiter for sharing UDP TX buffer between user and ROS2rapper IP
localparam UDP_TXBUF_GRANT_IP   = 1'b0;
localparam UDP_TXBUF_GRANT_USER = 1'b1;

reg r_udp_txbuf_grant;
wire udp_txbuf_ip_rel, udp_txbuf_ip_grant;
assign udp_txbuf_ip_grant = en & (~r_udp_txbuf_grant);
assign udp_txbuf_grant = en & r_udp_txbuf_grant;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_udp_txbuf_grant <= UDP_TXBUF_GRANT_USER;
    end else begin
        case (r_udp_txbuf_grant)
            UDP_TXBUF_GRANT_IP:
                if (udp_txbuf_ip_rel) r_udp_txbuf_grant <= UDP_TXBUF_GRANT_USER;
            UDP_TXBUF_GRANT_USER:
                if (udp_txbuf_rel) r_udp_txbuf_grant <= UDP_TXBUF_GRANT_IP;
        endcase
    end
end

`ifdef ROS2RAPPER_HLS_VITIS
ros2
ros2 (
    .ap_clk(clk),
    .ap_rst_n(rst_n),

    .pub_enable(en & ros2pub_en),
    .sub_enable(en & ros2sub_en),

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

    .ip_payloads_address0(ip_payloadsmem_addr),
    .ip_payloads_ce0(ip_payloadsmem_ce),
    .ip_payloads_we0(ip_payloadsmem_we),
    .ip_payloads_d0(ip_payloadsmem_wdata),
    .ip_payloads_q0(ip_payloadsmem_rdata),

    .conf_ip_addr(ip_addr),
    .conf_subnet_mask(subnet_mask),
    .conf_node_name(ros2_node_name),
    .conf_node_name_len(ros2_node_name_len),
    .conf_node_udp_port({ros2_node_udp_port[7:0], ros2_node_udp_port[15:8]}),
    .conf_rx_udp_port({ros2_rx_udp_port[7:0], ros2_rx_udp_port[15:8]}),
    .conf_port_num_seed(ros2_port_num_seed),
    .conf_tx_period(ros2_tx_period),
    .conf_fragment_expiration(ros2_fragment_expiration),
    .conf_guid_prefix(ros2_guid_prefix),
    .conf_ignore_ip_checksum(ros2_ignore_ip_checksum),

    .conf_pub_topic_name(ros2_pub_topic_name),
    .conf_pub_topic_name_len(ros2_pub_topic_name_len),
    .conf_pub_topic_type_name(ros2_pub_topic_type_name),
    .conf_pub_topic_type_name_len(ros2_pub_topic_type_name_len),
    .conf_sub_topic_name(ros2_sub_topic_name),
    .conf_sub_topic_name_len(ros2_sub_topic_name_len),
    .conf_sub_topic_type_name(ros2_sub_topic_type_name),
    .conf_sub_topic_type_name_len(ros2_sub_topic_type_name_len),

    .pub_app_data_dout(ros2_pub_app_data),
    .pub_app_data_empty_n(1'b1),
    .pub_app_data_read(),
    .pub_app_data_len_dout(ros2_pub_app_data_len),
    .pub_app_data_len_empty_n(1'b1),
    .pub_app_data_len_read(),
    .pub_app_data_req_ap_vld(ros2_pub_app_data_ip_req),
    .pub_app_data_req(),
    .pub_app_data_rel_ap_vld(ros2_pub_app_data_ip_rel),
    .pub_app_data_rel(),
    .pub_app_data_grant({7'b0, ros2_pub_app_data_ip_grant}),

    .sub_app_data_recv_ap_vld(ros2_sub_app_data_recv),
    .sub_app_data_recv(),
    .sub_app_data_grant({7'b0, ros2_sub_app_data_ip_grant}),
    .sub_app_data_address0(ros2_sub_app_data_addr),
    .sub_app_data_ce0(ros2_sub_app_data_ce),
    .sub_app_data_we0(ros2_sub_app_data_we),
    .sub_app_data_d0(ros2_sub_app_data_wdata),
    .sub_app_data_len(ros2_sub_app_data_len),
    .sub_app_data_rep_id(ros2_sub_app_data_rep_id),

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

  .pub_enable(en & ros2pub_en),
  .sub_enable(en & ros2sub_en),

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

  .ip_payloads_CS1(ip_payloadsmem_ce),
  .ip_payloads_AD1(ip_payloadsmem_addr),
  .ip_payloads_WE1(ip_payloadsmem_we),
  .ip_payloads_WD1(ip_payloadsmem_wdata),
  .ip_payloads_RD1(ip_payloadsmem_rdata),

  .conf_ip_addr_0(ip_addr[7:0]), .conf_ip_addr_1(ip_addr[15:8]),
  .conf_ip_addr_2(ip_addr[23:16]), .conf_ip_addr_3(ip_addr[31:24]),
  .conf_subnet_mask_0(subnet_mask[7:0]), .conf_subnet_mask_1(subnet_mask[15:8]),
  .conf_subnet_mask_2(subnet_mask[23:16]), .conf_subnet_mask_3(subnet_mask[31:24]),
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
  .conf_rx_udp_port_1(ros2_rx_udp_port[7:0]), .conf_rx_udp_port_0(ros2_rx_udp_port[15:8]),
  .conf_port_num_seed(ros2_port_num_seed),
  .conf_tx_period(ros2_tx_period),
  .conf_fragment_expiration(ros2_fragment_expiration),
  .conf_guid_prefix_00(ros2_guid_prefix[7:0]), .conf_guid_prefix_01(ros2_guid_prefix[15:8]),
  .conf_guid_prefix_02(ros2_guid_prefix[23:16]), .conf_guid_prefix_03(ros2_guid_prefix[31:24]),
  .conf_guid_prefix_04(ros2_guid_prefix[39:32]), .conf_guid_prefix_05(ros2_guid_prefix[47:40]),
  .conf_guid_prefix_06(ros2_guid_prefix[55:48]), .conf_guid_prefix_07(ros2_guid_prefix[63:56]),
  .conf_guid_prefix_08(ros2_guid_prefix[71:64]), .conf_guid_prefix_09(ros2_guid_prefix[79:72]),
  .conf_guid_prefix_10(ros2_guid_prefix[87:80]), .conf_guid_prefix_11(ros2_guid_prefix[95:88]),
  .conf_ignore_ip_checksum(ros2_ignore_ip_checksum),

  .conf_pub_topic_name_00(ros2_pub_topic_name[7:0]), .conf_pub_topic_name_01(ros2_pub_topic_name[15:8]),
  .conf_pub_topic_name_02(ros2_pub_topic_name[23:16]), .conf_pub_topic_name_03(ros2_pub_topic_name[31:24]),
  .conf_pub_topic_name_04(ros2_pub_topic_name[39:32]), .conf_pub_topic_name_05(ros2_pub_topic_name[47:40]),
  .conf_pub_topic_name_06(ros2_pub_topic_name[55:48]), .conf_pub_topic_name_07(ros2_pub_topic_name[63:56]),
  .conf_pub_topic_name_08(ros2_pub_topic_name[71:64]), .conf_pub_topic_name_09(ros2_pub_topic_name[79:72]),
  .conf_pub_topic_name_10(ros2_pub_topic_name[87:80]), .conf_pub_topic_name_11(ros2_pub_topic_name[95:88]),
  .conf_pub_topic_name_12(ros2_pub_topic_name[103:96]), .conf_pub_topic_name_13(ros2_pub_topic_name[111:104]),
  .conf_pub_topic_name_14(ros2_pub_topic_name[119:112]), .conf_pub_topic_name_15(ros2_pub_topic_name[127:120]),
  .conf_pub_topic_name_16(ros2_pub_topic_name[135:128]), .conf_pub_topic_name_17(ros2_pub_topic_name[143:136]),
  .conf_pub_topic_name_18(ros2_pub_topic_name[151:144]), .conf_pub_topic_name_19(ros2_pub_topic_name[159:152]),
  .conf_pub_topic_name_20(ros2_pub_topic_name[167:160]), .conf_pub_topic_name_21(ros2_pub_topic_name[175:168]),
  .conf_pub_topic_name_22(ros2_pub_topic_name[183:176]), .conf_pub_topic_name_23(ros2_pub_topic_name[191:184]),
  .conf_pub_topic_name_24(ros2_pub_topic_name[199:192]), .conf_pub_topic_name_25(ros2_pub_topic_name[207:200]),
  .conf_pub_topic_name_26(ros2_pub_topic_name[215:208]), .conf_pub_topic_name_27(ros2_pub_topic_name[223:216]),
  .conf_pub_topic_name_28(ros2_pub_topic_name[231:224]), .conf_pub_topic_name_29(ros2_pub_topic_name[239:232]),
  .conf_pub_topic_name_30(ros2_pub_topic_name[247:240]), .conf_pub_topic_name_31(ros2_pub_topic_name[255:248]),
  .conf_pub_topic_name_len(ros2_pub_topic_name_len),
  .conf_pub_topic_type_name_00(ros2_pub_topic_type_name[7:0]), .conf_pub_topic_type_name_01(ros2_pub_topic_type_name[15:8]),
  .conf_pub_topic_type_name_02(ros2_pub_topic_type_name[23:16]), .conf_pub_topic_type_name_03(ros2_pub_topic_type_name[31:24]),
  .conf_pub_topic_type_name_04(ros2_pub_topic_type_name[39:32]), .conf_pub_topic_type_name_05(ros2_pub_topic_type_name[47:40]),
  .conf_pub_topic_type_name_06(ros2_pub_topic_type_name[55:48]), .conf_pub_topic_type_name_07(ros2_pub_topic_type_name[63:56]),
  .conf_pub_topic_type_name_08(ros2_pub_topic_type_name[71:64]), .conf_pub_topic_type_name_09(ros2_pub_topic_type_name[79:72]),
  .conf_pub_topic_type_name_10(ros2_pub_topic_type_name[87:80]), .conf_pub_topic_type_name_11(ros2_pub_topic_type_name[95:88]),
  .conf_pub_topic_type_name_12(ros2_pub_topic_type_name[103:96]), .conf_pub_topic_type_name_13(ros2_pub_topic_type_name[111:104]),
  .conf_pub_topic_type_name_14(ros2_pub_topic_type_name[119:112]), .conf_pub_topic_type_name_15(ros2_pub_topic_type_name[127:120]),
  .conf_pub_topic_type_name_16(ros2_pub_topic_type_name[135:128]), .conf_pub_topic_type_name_17(ros2_pub_topic_type_name[143:136]),
  .conf_pub_topic_type_name_18(ros2_pub_topic_type_name[151:144]), .conf_pub_topic_type_name_19(ros2_pub_topic_type_name[159:152]),
  .conf_pub_topic_type_name_20(ros2_pub_topic_type_name[167:160]), .conf_pub_topic_type_name_21(ros2_pub_topic_type_name[175:168]),
  .conf_pub_topic_type_name_22(ros2_pub_topic_type_name[183:176]), .conf_pub_topic_type_name_23(ros2_pub_topic_type_name[191:184]),
  .conf_pub_topic_type_name_24(ros2_pub_topic_type_name[199:192]), .conf_pub_topic_type_name_25(ros2_pub_topic_type_name[207:200]),
  .conf_pub_topic_type_name_26(ros2_pub_topic_type_name[215:208]), .conf_pub_topic_type_name_27(ros2_pub_topic_type_name[223:216]),
  .conf_pub_topic_type_name_28(ros2_pub_topic_type_name[231:224]), .conf_pub_topic_type_name_29(ros2_pub_topic_type_name[239:232]),
  .conf_pub_topic_type_name_30(ros2_pub_topic_type_name[247:240]), .conf_pub_topic_type_name_31(ros2_pub_topic_type_name[255:248]),
  .conf_pub_topic_type_name_32(ros2_pub_topic_type_name[263:256]), .conf_pub_topic_type_name_33(ros2_pub_topic_type_name[271:264]),
  .conf_pub_topic_type_name_34(ros2_pub_topic_type_name[279:272]), .conf_pub_topic_type_name_35(ros2_pub_topic_type_name[287:280]),
  .conf_pub_topic_type_name_36(ros2_pub_topic_type_name[295:288]), .conf_pub_topic_type_name_37(ros2_pub_topic_type_name[303:296]),
  .conf_pub_topic_type_name_38(ros2_pub_topic_type_name[311:304]), .conf_pub_topic_type_name_39(ros2_pub_topic_type_name[319:312]),
  .conf_pub_topic_type_name_40(ros2_pub_topic_type_name[327:320]), .conf_pub_topic_type_name_41(ros2_pub_topic_type_name[335:328]),
  .conf_pub_topic_type_name_42(ros2_pub_topic_type_name[343:336]), .conf_pub_topic_type_name_43(ros2_pub_topic_type_name[351:344]),
  .conf_pub_topic_type_name_44(ros2_pub_topic_type_name[359:352]), .conf_pub_topic_type_name_45(ros2_pub_topic_type_name[367:360]),
  .conf_pub_topic_type_name_46(ros2_pub_topic_type_name[375:368]), .conf_pub_topic_type_name_47(ros2_pub_topic_type_name[383:376]),
  .conf_pub_topic_type_name_48(ros2_pub_topic_type_name[391:384]), .conf_pub_topic_type_name_49(ros2_pub_topic_type_name[399:392]),
  .conf_pub_topic_type_name_50(ros2_pub_topic_type_name[407:400]), .conf_pub_topic_type_name_51(ros2_pub_topic_type_name[415:408]),
  .conf_pub_topic_type_name_52(ros2_pub_topic_type_name[423:416]), .conf_pub_topic_type_name_53(ros2_pub_topic_type_name[431:424]),
  .conf_pub_topic_type_name_54(ros2_pub_topic_type_name[439:432]), .conf_pub_topic_type_name_55(ros2_pub_topic_type_name[447:440]),
  .conf_pub_topic_type_name_56(ros2_pub_topic_type_name[455:448]), .conf_pub_topic_type_name_57(ros2_pub_topic_type_name[463:456]),
  .conf_pub_topic_type_name_58(ros2_pub_topic_type_name[471:464]), .conf_pub_topic_type_name_59(ros2_pub_topic_type_name[479:472]),
  .conf_pub_topic_type_name_60(ros2_pub_topic_type_name[487:480]), .conf_pub_topic_type_name_61(ros2_pub_topic_type_name[495:488]),
  .conf_pub_topic_type_name_62(ros2_pub_topic_type_name[503:496]), .conf_pub_topic_type_name_63(ros2_pub_topic_type_name[511:504]),
  .conf_pub_topic_type_name_len(ros2_pub_topic_type_name_len),

  .conf_sub_topic_name_00(ros2_sub_topic_name[7:0]), .conf_sub_topic_name_01(ros2_sub_topic_name[15:8]),
  .conf_sub_topic_name_02(ros2_sub_topic_name[23:16]), .conf_sub_topic_name_03(ros2_sub_topic_name[31:24]),
  .conf_sub_topic_name_04(ros2_sub_topic_name[39:32]), .conf_sub_topic_name_05(ros2_sub_topic_name[47:40]),
  .conf_sub_topic_name_06(ros2_sub_topic_name[55:48]), .conf_sub_topic_name_07(ros2_sub_topic_name[63:56]),
  .conf_sub_topic_name_08(ros2_sub_topic_name[71:64]), .conf_sub_topic_name_09(ros2_sub_topic_name[79:72]),
  .conf_sub_topic_name_10(ros2_sub_topic_name[87:80]), .conf_sub_topic_name_11(ros2_sub_topic_name[95:88]),
  .conf_sub_topic_name_12(ros2_sub_topic_name[103:96]), .conf_sub_topic_name_13(ros2_sub_topic_name[111:104]),
  .conf_sub_topic_name_14(ros2_sub_topic_name[119:112]), .conf_sub_topic_name_15(ros2_sub_topic_name[127:120]),
  .conf_sub_topic_name_16(ros2_sub_topic_name[135:128]), .conf_sub_topic_name_17(ros2_sub_topic_name[143:136]),
  .conf_sub_topic_name_18(ros2_sub_topic_name[151:144]), .conf_sub_topic_name_19(ros2_sub_topic_name[159:152]),
  .conf_sub_topic_name_20(ros2_sub_topic_name[167:160]), .conf_sub_topic_name_21(ros2_sub_topic_name[175:168]),
  .conf_sub_topic_name_22(ros2_sub_topic_name[183:176]), .conf_sub_topic_name_23(ros2_sub_topic_name[191:184]),
  .conf_sub_topic_name_24(ros2_sub_topic_name[199:192]), .conf_sub_topic_name_25(ros2_sub_topic_name[207:200]),
  .conf_sub_topic_name_26(ros2_sub_topic_name[215:208]), .conf_sub_topic_name_27(ros2_sub_topic_name[223:216]),
  .conf_sub_topic_name_28(ros2_sub_topic_name[231:224]), .conf_sub_topic_name_29(ros2_sub_topic_name[239:232]),
  .conf_sub_topic_name_30(ros2_sub_topic_name[247:240]), .conf_sub_topic_name_31(ros2_sub_topic_name[255:248]),
  .conf_sub_topic_name_len(ros2_sub_topic_name_len),
  .conf_sub_topic_type_name_00(ros2_sub_topic_type_name[7:0]), .conf_sub_topic_type_name_01(ros2_sub_topic_type_name[15:8]),
  .conf_sub_topic_type_name_02(ros2_sub_topic_type_name[23:16]), .conf_sub_topic_type_name_03(ros2_sub_topic_type_name[31:24]),
  .conf_sub_topic_type_name_04(ros2_sub_topic_type_name[39:32]), .conf_sub_topic_type_name_05(ros2_sub_topic_type_name[47:40]),
  .conf_sub_topic_type_name_06(ros2_sub_topic_type_name[55:48]), .conf_sub_topic_type_name_07(ros2_sub_topic_type_name[63:56]),
  .conf_sub_topic_type_name_08(ros2_sub_topic_type_name[71:64]), .conf_sub_topic_type_name_09(ros2_sub_topic_type_name[79:72]),
  .conf_sub_topic_type_name_10(ros2_sub_topic_type_name[87:80]), .conf_sub_topic_type_name_11(ros2_sub_topic_type_name[95:88]),
  .conf_sub_topic_type_name_12(ros2_sub_topic_type_name[103:96]), .conf_sub_topic_type_name_13(ros2_sub_topic_type_name[111:104]),
  .conf_sub_topic_type_name_14(ros2_sub_topic_type_name[119:112]), .conf_sub_topic_type_name_15(ros2_sub_topic_type_name[127:120]),
  .conf_sub_topic_type_name_16(ros2_sub_topic_type_name[135:128]), .conf_sub_topic_type_name_17(ros2_sub_topic_type_name[143:136]),
  .conf_sub_topic_type_name_18(ros2_sub_topic_type_name[151:144]), .conf_sub_topic_type_name_19(ros2_sub_topic_type_name[159:152]),
  .conf_sub_topic_type_name_20(ros2_sub_topic_type_name[167:160]), .conf_sub_topic_type_name_21(ros2_sub_topic_type_name[175:168]),
  .conf_sub_topic_type_name_22(ros2_sub_topic_type_name[183:176]), .conf_sub_topic_type_name_23(ros2_sub_topic_type_name[191:184]),
  .conf_sub_topic_type_name_24(ros2_sub_topic_type_name[199:192]), .conf_sub_topic_type_name_25(ros2_sub_topic_type_name[207:200]),
  .conf_sub_topic_type_name_26(ros2_sub_topic_type_name[215:208]), .conf_sub_topic_type_name_27(ros2_sub_topic_type_name[223:216]),
  .conf_sub_topic_type_name_28(ros2_sub_topic_type_name[231:224]), .conf_sub_topic_type_name_29(ros2_sub_topic_type_name[239:232]),
  .conf_sub_topic_type_name_30(ros2_sub_topic_type_name[247:240]), .conf_sub_topic_type_name_31(ros2_sub_topic_type_name[255:248]),
  .conf_sub_topic_type_name_32(ros2_sub_topic_type_name[263:256]), .conf_sub_topic_type_name_33(ros2_sub_topic_type_name[271:264]),
  .conf_sub_topic_type_name_34(ros2_sub_topic_type_name[279:272]), .conf_sub_topic_type_name_35(ros2_sub_topic_type_name[287:280]),
  .conf_sub_topic_type_name_36(ros2_sub_topic_type_name[295:288]), .conf_sub_topic_type_name_37(ros2_sub_topic_type_name[303:296]),
  .conf_sub_topic_type_name_38(ros2_sub_topic_type_name[311:304]), .conf_sub_topic_type_name_39(ros2_sub_topic_type_name[319:312]),
  .conf_sub_topic_type_name_40(ros2_sub_topic_type_name[327:320]), .conf_sub_topic_type_name_41(ros2_sub_topic_type_name[335:328]),
  .conf_sub_topic_type_name_42(ros2_sub_topic_type_name[343:336]), .conf_sub_topic_type_name_43(ros2_sub_topic_type_name[351:344]),
  .conf_sub_topic_type_name_44(ros2_sub_topic_type_name[359:352]), .conf_sub_topic_type_name_45(ros2_sub_topic_type_name[367:360]),
  .conf_sub_topic_type_name_46(ros2_sub_topic_type_name[375:368]), .conf_sub_topic_type_name_47(ros2_sub_topic_type_name[383:376]),
  .conf_sub_topic_type_name_48(ros2_sub_topic_type_name[391:384]), .conf_sub_topic_type_name_49(ros2_sub_topic_type_name[399:392]),
  .conf_sub_topic_type_name_50(ros2_sub_topic_type_name[407:400]), .conf_sub_topic_type_name_51(ros2_sub_topic_type_name[415:408]),
  .conf_sub_topic_type_name_52(ros2_sub_topic_type_name[423:416]), .conf_sub_topic_type_name_53(ros2_sub_topic_type_name[431:424]),
  .conf_sub_topic_type_name_54(ros2_sub_topic_type_name[439:432]), .conf_sub_topic_type_name_55(ros2_sub_topic_type_name[447:440]),
  .conf_sub_topic_type_name_56(ros2_sub_topic_type_name[455:448]), .conf_sub_topic_type_name_57(ros2_sub_topic_type_name[463:456]),
  .conf_sub_topic_type_name_58(ros2_sub_topic_type_name[471:464]), .conf_sub_topic_type_name_59(ros2_sub_topic_type_name[479:472]),
  .conf_sub_topic_type_name_60(ros2_sub_topic_type_name[487:480]), .conf_sub_topic_type_name_61(ros2_sub_topic_type_name[495:488]),
  .conf_sub_topic_type_name_62(ros2_sub_topic_type_name[503:496]), .conf_sub_topic_type_name_63(ros2_sub_topic_type_name[511:504]),
  .conf_sub_topic_type_name_len(ros2_sub_topic_type_name_len),

  .pub_app_data_00_rd(ros2_pub_app_data[7:0]),     .pub_app_data_01_rd(ros2_pub_app_data[15:8]),
  .pub_app_data_02_rd(ros2_pub_app_data[23:16]),   .pub_app_data_03_rd(ros2_pub_app_data[31:24]),
  .pub_app_data_04_rd(ros2_pub_app_data[39:32]),   .pub_app_data_05_rd(ros2_pub_app_data[47:40]),
  .pub_app_data_06_rd(ros2_pub_app_data[55:48]),   .pub_app_data_07_rd(ros2_pub_app_data[63:56]),
  .pub_app_data_08_rd(ros2_pub_app_data[71:64]),   .pub_app_data_09_rd(ros2_pub_app_data[79:72]),
  .pub_app_data_10_rd(ros2_pub_app_data[87:80]),   .pub_app_data_11_rd(ros2_pub_app_data[95:88]),
  .pub_app_data_12_rd(ros2_pub_app_data[103:96]),  .pub_app_data_13_rd(ros2_pub_app_data[111:104]),
  .pub_app_data_14_rd(ros2_pub_app_data[119:112]), .pub_app_data_15_rd(ros2_pub_app_data[127:120]),
  .pub_app_data_16_rd(ros2_pub_app_data[135:128]), .pub_app_data_17_rd(ros2_pub_app_data[143:136]),
  .pub_app_data_18_rd(ros2_pub_app_data[151:144]), .pub_app_data_19_rd(ros2_pub_app_data[159:152]),
  .pub_app_data_20_rd(ros2_pub_app_data[167:160]), .pub_app_data_21_rd(ros2_pub_app_data[175:168]),
  .pub_app_data_22_rd(ros2_pub_app_data[183:176]), .pub_app_data_23_rd(ros2_pub_app_data[191:184]),
  .pub_app_data_24_rd(ros2_pub_app_data[199:192]), .pub_app_data_25_rd(ros2_pub_app_data[207:200]),
  .pub_app_data_26_rd(ros2_pub_app_data[215:208]), .pub_app_data_27_rd(ros2_pub_app_data[223:216]),
  .pub_app_data_28_rd(ros2_pub_app_data[231:224]), .pub_app_data_29_rd(ros2_pub_app_data[239:232]),
  .pub_app_data_30_rd(ros2_pub_app_data[247:240]), .pub_app_data_31_rd(ros2_pub_app_data[255:248]),
  .pub_app_data_32_rd(ros2_pub_app_data[263:256]), .pub_app_data_33_rd(ros2_pub_app_data[271:264]),
  .pub_app_data_34_rd(ros2_pub_app_data[279:272]), .pub_app_data_35_rd(ros2_pub_app_data[287:280]),
  .pub_app_data_36_rd(ros2_pub_app_data[295:288]), .pub_app_data_37_rd(ros2_pub_app_data[303:296]),
  .pub_app_data_38_rd(ros2_pub_app_data[311:304]), .pub_app_data_39_rd(ros2_pub_app_data[319:312]),
  .pub_app_data_40_rd(ros2_pub_app_data[327:320]), .pub_app_data_41_rd(ros2_pub_app_data[335:328]),
  .pub_app_data_42_rd(ros2_pub_app_data[343:336]), .pub_app_data_43_rd(ros2_pub_app_data[351:344]),
  .pub_app_data_44_rd(ros2_pub_app_data[359:352]), .pub_app_data_45_rd(ros2_pub_app_data[367:360]),
  .pub_app_data_46_rd(ros2_pub_app_data[375:368]), .pub_app_data_47_rd(ros2_pub_app_data[383:376]),
  .pub_app_data_48_rd(ros2_pub_app_data[391:384]), .pub_app_data_49_rd(ros2_pub_app_data[399:392]),
  .pub_app_data_50_rd(ros2_pub_app_data[407:400]), .pub_app_data_51_rd(ros2_pub_app_data[415:408]),
  .pub_app_data_52_rd(ros2_pub_app_data[423:416]), .pub_app_data_53_rd(ros2_pub_app_data[431:424]),
  .pub_app_data_54_rd(ros2_pub_app_data[439:432]), .pub_app_data_55_rd(ros2_pub_app_data[447:440]),
  .pub_app_data_56_rd(ros2_pub_app_data[455:448]), .pub_app_data_57_rd(ros2_pub_app_data[463:456]),
  .pub_app_data_58_rd(ros2_pub_app_data[471:464]), .pub_app_data_59_rd(ros2_pub_app_data[479:472]),
  .pub_app_data_60_rd(ros2_pub_app_data[487:480]), .pub_app_data_61_rd(ros2_pub_app_data[495:488]),
  .pub_app_data_62_rd(ros2_pub_app_data[503:496]), .pub_app_data_63_rd(ros2_pub_app_data[511:504]),
  .pub_app_data_len_rreq(),
  .pub_app_data_len_empty(1'b0),
  .pub_app_data_len_dout(ros2_pub_app_data_len),
  .pub_app_data_req_we(ros2_pub_app_data_ip_req),
  .pub_app_data_req_wd(),
  .pub_app_data_rel_we(ros2_pub_app_data_ip_rel),
  .pub_app_data_rel_wd(),
  .pub_app_data_grant_rd({7'b0, ros2_pub_app_data_ip_grant}),

  .sub_app_data_CS1(ros2_sub_app_data_ce),
  .sub_app_data_AD1(ros2_sub_app_data_addr),
  .sub_app_data_WE1(ros2_sub_app_data_we),
  .sub_app_data_WD1(ros2_sub_app_data_wdata),
  .sub_app_data_len(ros2_sub_app_data_len),
  .sub_app_data_rep_id(ros2_sub_app_data_rep_id),
  .sub_app_data_recv_we(ros2_sub_app_data_recv),
  .sub_app_data_recv_wd(),
  .sub_app_data_grant_rd({7'b0, ros2_sub_app_data_ip_grant}),

  .udp_rxbuf_rel_we(udp_rxbuf_ip_rel),
  .udp_rxbuf_rel_wd(),
  .udp_rxbuf_grant_rd({7'b0, udp_rxbuf_ip_grant}),

  .udp_txbuf_rel_we(udp_txbuf_ip_rel),
  .udp_txbuf_rel_wd(),
  .udp_txbuf_grant_rd({7'b0, udp_txbuf_ip_grant})
);
`endif

endmodule

`resetall
