// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none

`include "ros2_config.vh"

module ros2rapper #(
    parameter PRESCALER_DIV               = 64,
    parameter ROS2CLK_HZ                  = 100_000_000,
    parameter TX_INTERVAL_COUNT           = (ROS2CLK_HZ / PRESCALER_DIV) / 100,
    parameter TX_PERIOD_SPDP_WR_COUNT     = (ROS2CLK_HZ / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_PUB_WR_COUNT = (ROS2CLK_HZ / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_SUB_WR_COUNT = (ROS2CLK_HZ / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_PUB_HB_COUNT = (ROS2CLK_HZ / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_SUB_HB_COUNT = (ROS2CLK_HZ / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_PUB_AN_COUNT = (ROS2CLK_HZ / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_SUB_AN_COUNT = (ROS2CLK_HZ / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_APP_WR_COUNT      = (ROS2CLK_HZ / PRESCALER_DIV) * 3
)
(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       en,
    input  wire       ros2pub_en,
    input  wire [3:0] ros2sub_en,

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

    input  wire [31:0] ros2_fragment_expiration,
    input  wire [95:0] ros2_guid_prefix,
    input  wire [31:0] ros2_participant_lease_duration_seconds,
    input  wire [31:0] ros2_participant_lease_duration_fraction,
    input  wire        ros2_ignore_ip_checksum,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name,
    input  wire [7:0] ros2_pub_topic_name_len,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name,
    input  wire [7:0] ros2_pub_topic_type_name_len,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name_0,
    input  wire [7:0] ros2_sub_topic_name_len_0,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name_0,
    input  wire [7:0] ros2_sub_topic_type_name_len_0,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name_1,
    input  wire [7:0] ros2_sub_topic_name_len_1,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name_1,
    input  wire [7:0] ros2_sub_topic_type_name_len_1,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name_2,
    input  wire [7:0] ros2_sub_topic_name_len_2,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name_2,
    input  wire [7:0] ros2_sub_topic_type_name_len_2,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name_3,
    input  wire [7:0] ros2_sub_topic_name_len_3,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name_3,
    input  wire [7:0] ros2_sub_topic_type_name_len_3,

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
    output wire [3:0] ros2_sub_app_data_recv,

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
    APP_DATA_GRANT_NONE = 2'b00,
    APP_DATA_GRANT_IP   = 2'b01,
    APP_DATA_GRANT_USER = 2'b10;

reg [1:0] r_ros2_pub_app_data_grant;
wire ros2_pub_app_data_ip_req, ros2_pub_app_data_ip_rel, ros2_pub_app_data_ip_grant;
assign ros2_pub_app_data_ip_grant = en & r_ros2_pub_app_data_grant[0];
assign ros2_pub_app_data_grant = en & r_ros2_pub_app_data_grant[1];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_ros2_pub_app_data_grant <= APP_DATA_GRANT_NONE;
    end else begin
        case (r_ros2_pub_app_data_grant)
            APP_DATA_GRANT_NONE: begin
                case ({ros2_pub_app_data_ip_req, ros2_pub_app_data_req})
                    2'b00: r_ros2_pub_app_data_grant <= APP_DATA_GRANT_NONE;
                    2'b01: r_ros2_pub_app_data_grant <= APP_DATA_GRANT_USER;
                    2'b10: r_ros2_pub_app_data_grant <= APP_DATA_GRANT_IP;
                    2'b11: r_ros2_pub_app_data_grant <= APP_DATA_GRANT_IP;
                endcase
            end
            APP_DATA_GRANT_IP:
                if (ros2_pub_app_data_ip_rel) r_ros2_pub_app_data_grant <= APP_DATA_GRANT_NONE;
            APP_DATA_GRANT_USER:
                if (ros2_pub_app_data_rel) r_ros2_pub_app_data_grant <= APP_DATA_GRANT_NONE;
            default:
                r_ros2_pub_app_data_grant <= APP_DATA_GRANT_NONE;
        endcase
    end
end

// arbiter for sharing subscriber app_data buffer between user and IP
reg [1:0] r_ros2_sub_app_data_grant;
wire ros2_sub_app_data_ip_req, ros2_sub_app_data_ip_rel, ros2_sub_app_data_ip_grant;
assign ros2_sub_app_data_ip_grant = (en && (ros2sub_en != 0)) & r_ros2_sub_app_data_grant[0];
assign ros2_sub_app_data_grant = (en && (ros2sub_en != 0)) & r_ros2_sub_app_data_grant[1];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_ros2_sub_app_data_grant <= APP_DATA_GRANT_NONE;
    end else begin
        case (r_ros2_sub_app_data_grant)
            APP_DATA_GRANT_NONE: begin
                case ({ros2_sub_app_data_ip_req, ros2_sub_app_data_req})
                    2'b00: r_ros2_sub_app_data_grant <= APP_DATA_GRANT_NONE;
                    2'b01: r_ros2_sub_app_data_grant <= APP_DATA_GRANT_USER;
                    2'b10: r_ros2_sub_app_data_grant <= APP_DATA_GRANT_IP;
                    2'b11: r_ros2_sub_app_data_grant <= APP_DATA_GRANT_IP;
                endcase
            end
            APP_DATA_GRANT_IP:
                if (ros2_sub_app_data_ip_rel) r_ros2_sub_app_data_grant <= APP_DATA_GRANT_NONE;
            APP_DATA_GRANT_USER:
                if (ros2_sub_app_data_rel) r_ros2_sub_app_data_grant <= APP_DATA_GRANT_NONE;
            default:
                r_ros2_sub_app_data_grant <= APP_DATA_GRANT_NONE;
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

// local_timestamp[63:32] is time in second and local_timestamp[31:0] is the fractional part.
reg [63:0] local_timestamp;
// Designate the bit length to prevent overflow.
localparam [33:0] TWO_SECONDS = 2 * (2 ** 32);
// How much local_timestamp increases in each cycle.
localparam LOCAL_TIMESTAMP_INCREMENT = (TWO_SECONDS + ROS2CLK_HZ) / (2 * ROS2CLK_HZ);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        local_timestamp <= 64'd0;
    end else begin
        local_timestamp <= local_timestamp + LOCAL_TIMESTAMP_INCREMENT;
    end
end

wire [3:0] sub_app_data_recv;
wire       sub_app_data_recv_ap_vld;
assign ros2_sub_app_data_recv[0] = sub_app_data_recv_ap_vld & sub_app_data_recv[0];
assign ros2_sub_app_data_recv[1] = sub_app_data_recv_ap_vld & sub_app_data_recv[1];
assign ros2_sub_app_data_recv[2] = sub_app_data_recv_ap_vld & sub_app_data_recv[2];
assign ros2_sub_app_data_recv[3] = sub_app_data_recv_ap_vld & sub_app_data_recv[3];

wire ros2_cnt_interval_set;
wire ros2_cnt_spdp_wr_set;
wire ros2_cnt_sedp_pub_wr_set;
wire ros2_cnt_sedp_sub_wr_set;
wire ros2_cnt_sedp_pub_hb_set;
wire ros2_cnt_sedp_sub_hb_set;
wire ros2_cnt_sedp_pub_an_set;
wire ros2_cnt_sedp_sub_an_set;
wire ros2_cnt_app_wr_set;

wire ros2_cnt_interval_elapsed;
wire ros2_cnt_spdp_wr_elapsed;
wire ros2_cnt_sedp_pub_wr_elapsed;
wire ros2_cnt_sedp_sub_wr_elapsed;
wire ros2_cnt_sedp_pub_hb_elapsed;
wire ros2_cnt_sedp_sub_hb_elapsed;
wire ros2_cnt_sedp_pub_an_elapsed;
wire ros2_cnt_sedp_sub_an_elapsed;
wire ros2_cnt_app_wr_elapsed;

ros2rapper_tx_counters #(
    .PRESCALER_DIV              (PRESCALER_DIV              ),
    .TX_INTERVAL_COUNT          (TX_INTERVAL_COUNT          ),
    .TX_PERIOD_SPDP_WR_COUNT    (TX_PERIOD_SPDP_WR_COUNT    ),
    .TX_PERIOD_SEDP_PUB_WR_COUNT(TX_PERIOD_SEDP_PUB_WR_COUNT),
    .TX_PERIOD_SEDP_SUB_WR_COUNT(TX_PERIOD_SEDP_SUB_WR_COUNT),
    .TX_PERIOD_SEDP_PUB_HB_COUNT(TX_PERIOD_SEDP_PUB_HB_COUNT),
    .TX_PERIOD_SEDP_SUB_HB_COUNT(TX_PERIOD_SEDP_SUB_HB_COUNT),
    .TX_PERIOD_SEDP_PUB_AN_COUNT(TX_PERIOD_SEDP_PUB_AN_COUNT),
    .TX_PERIOD_SEDP_SUB_AN_COUNT(TX_PERIOD_SEDP_SUB_AN_COUNT),
    .TX_PERIOD_APP_WR_COUNT     (TX_PERIOD_APP_WR_COUNT     )
)
ros2rapper_tx_counters (
    .i_clk(clk),
    .i_rst_n(rst_n),

    .i_cnt_interval_set(ros2_cnt_interval_set),
    .i_cnt_spdp_wr_set(ros2_cnt_spdp_wr_set),
    .i_cnt_sedp_pub_wr_set(ros2_cnt_sedp_pub_wr_set),
    .i_cnt_sedp_sub_wr_set(ros2_cnt_sedp_sub_wr_set),
    .i_cnt_sedp_pub_hb_set(ros2_cnt_sedp_pub_hb_set),
    .i_cnt_sedp_sub_hb_set(ros2_cnt_sedp_sub_hb_set),
    .i_cnt_sedp_pub_an_set(ros2_cnt_sedp_pub_an_set),
    .i_cnt_sedp_sub_an_set(ros2_cnt_sedp_sub_an_set),
    .i_cnt_app_wr_set(ros2_cnt_app_wr_set),

    .o_cnt_interval_elapsed(ros2_cnt_interval_elapsed),
    .o_cnt_spdp_wr_elapsed(ros2_cnt_spdp_wr_elapsed),
    .o_cnt_sedp_pub_wr_elapsed(ros2_cnt_sedp_pub_wr_elapsed),
    .o_cnt_sedp_sub_wr_elapsed(ros2_cnt_sedp_sub_wr_elapsed),
    .o_cnt_sedp_pub_hb_elapsed(ros2_cnt_sedp_pub_hb_elapsed),
    .o_cnt_sedp_sub_hb_elapsed(ros2_cnt_sedp_sub_hb_elapsed),
    .o_cnt_sedp_pub_an_elapsed(ros2_cnt_sedp_pub_an_elapsed),
    .o_cnt_sedp_sub_an_elapsed(ros2_cnt_sedp_sub_an_elapsed),
    .o_cnt_app_wr_elapsed(ros2_cnt_app_wr_elapsed)
);

`ifdef ROS2RAPPER_HLS_VITIS
ros2
ros2 (
    .ap_clk(clk),
    .ap_rst_n(rst_n),

    .pub_enable(en & ros2pub_en),
    .sub_enable(en ? ros2sub_en : 4'd0),

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

    .conf_fragment_expiration(ros2_fragment_expiration),
    .conf_guid_prefix(ros2_guid_prefix),
    .conf_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
    .conf_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),
    .conf_ignore_ip_checksum(ros2_ignore_ip_checksum),

    .conf_pub_topic_name(ros2_pub_topic_name),
    .conf_pub_topic_name_len(ros2_pub_topic_name_len),
    .conf_pub_topic_type_name(ros2_pub_topic_type_name),
    .conf_pub_topic_type_name_len(ros2_pub_topic_type_name_len),
    .conf_sub_topic_name_0(ros2_sub_topic_name_0),
    .conf_sub_topic_name_len_0(ros2_sub_topic_name_len_0),
    .conf_sub_topic_type_name_0(ros2_sub_topic_type_name_0),
    .conf_sub_topic_type_name_len_0(ros2_sub_topic_type_name_len_0),
    .conf_sub_topic_name_1(ros2_sub_topic_name_1),
    .conf_sub_topic_name_len_1(ros2_sub_topic_name_len_1),
    .conf_sub_topic_type_name_1(ros2_sub_topic_type_name_1),
    .conf_sub_topic_type_name_len_1(ros2_sub_topic_type_name_len_1),
    .conf_sub_topic_name_2(ros2_sub_topic_name_2),
    .conf_sub_topic_name_len_2(ros2_sub_topic_name_len_2),
    .conf_sub_topic_type_name_2(ros2_sub_topic_type_name_2),
    .conf_sub_topic_type_name_len_2(ros2_sub_topic_type_name_len_2),
    .conf_sub_topic_name_3(ros2_sub_topic_name_3),
    .conf_sub_topic_name_len_3(ros2_sub_topic_name_len_3),
    .conf_sub_topic_type_name_3(ros2_sub_topic_type_name_3),
    .conf_sub_topic_type_name_len_3(ros2_sub_topic_type_name_len_3),

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
    .pub_app_data_grant_ap_ack(),

    .sub_app_data_recv_ap_vld(sub_app_data_recv_ap_vld),
    .sub_app_data_recv(sub_app_data_recv),
    .sub_app_data_req_ap_vld(ros2_sub_app_data_ip_req),
    .sub_app_data_req(),
    .sub_app_data_rel_ap_vld(ros2_sub_app_data_ip_rel),
    .sub_app_data_rel(),
    .sub_app_data_grant({7'b0, ros2_sub_app_data_ip_grant}),
    .sub_app_data_grant_ap_ack(),
    .sub_app_data_address0(ros2_sub_app_data_addr),
    .sub_app_data_ce0(ros2_sub_app_data_ce),
    .sub_app_data_we0(ros2_sub_app_data_we),
    .sub_app_data_d0(ros2_sub_app_data_wdata),
    .sub_app_data_len(ros2_sub_app_data_len),
    .sub_app_data_rep_id(ros2_sub_app_data_rep_id),

    .cnt_interval_set(),
    .cnt_interval_set_ap_vld(ros2_cnt_interval_set),
    .cnt_spdp_wr_set(),
    .cnt_spdp_wr_set_ap_vld(ros2_cnt_spdp_wr_set),
    .cnt_sedp_pub_wr_set(),
    .cnt_sedp_pub_wr_set_ap_vld(ros2_cnt_sedp_pub_wr_set),
    .cnt_sedp_sub_wr_set(),
    .cnt_sedp_sub_wr_set_ap_vld(ros2_cnt_sedp_sub_wr_set),
    .cnt_sedp_pub_hb_set(),
    .cnt_sedp_pub_hb_set_ap_vld(ros2_cnt_sedp_pub_hb_set),
    .cnt_sedp_sub_hb_set(),
    .cnt_sedp_sub_hb_set_ap_vld(ros2_cnt_sedp_sub_hb_set),
    .cnt_sedp_pub_an_set(),
    .cnt_sedp_pub_an_set_ap_vld(ros2_cnt_sedp_pub_an_set),
    .cnt_sedp_sub_an_set(),
    .cnt_sedp_sub_an_set_ap_vld(ros2_cnt_sedp_sub_an_set),
    .cnt_app_wr_set(),
    .cnt_app_wr_set_ap_vld(ros2_cnt_app_wr_set),

    .cnt_interval_elapsed(ros2_cnt_interval_elapsed),
    .cnt_interval_elapsed_ap_ack(),
    .cnt_spdp_wr_elapsed(ros2_cnt_spdp_wr_elapsed),
    .cnt_spdp_wr_elapsed_ap_ack(),
    .cnt_sedp_pub_wr_elapsed(ros2_cnt_sedp_pub_wr_elapsed),
    .cnt_sedp_pub_wr_elapsed_ap_ack(),
    .cnt_sedp_sub_wr_elapsed(ros2_cnt_sedp_sub_wr_elapsed),
    .cnt_sedp_sub_wr_elapsed_ap_ack(),
    .cnt_sedp_pub_hb_elapsed(ros2_cnt_sedp_pub_hb_elapsed),
    .cnt_sedp_pub_hb_elapsed_ap_ack(),
    .cnt_sedp_sub_hb_elapsed(ros2_cnt_sedp_sub_hb_elapsed),
    .cnt_sedp_sub_hb_elapsed_ap_ack(),
    .cnt_sedp_pub_an_elapsed(ros2_cnt_sedp_pub_an_elapsed),
    .cnt_sedp_pub_an_elapsed_ap_ack(),
    .cnt_sedp_sub_an_elapsed(ros2_cnt_sedp_sub_an_elapsed),
    .cnt_sedp_sub_an_elapsed_ap_ack(),
    .cnt_app_wr_elapsed(ros2_cnt_app_wr_elapsed),
    .cnt_app_wr_elapsed_ap_ack(),

    .udp_rxbuf_rel_ap_vld(udp_rxbuf_ip_rel),
    .udp_rxbuf_rel(),
    .udp_rxbuf_grant({7'b0, udp_rxbuf_ip_grant}),
    .udp_rxbuf_grant_ap_ack(),

    .udp_txbuf_rel_ap_vld(udp_txbuf_ip_rel),
    .udp_txbuf_rel(),
    .udp_txbuf_grant({7'b0, udp_txbuf_ip_grant}),
    .udp_txbuf_grant_ap_ack(),

    .timestamp_i64(local_timestamp),

    .xout(),
    .xout_ap_vld()
);
`elsif ROS2RAPPER_HLS_CWB
ros2
ros2 (
  .clk(clk),
  .rst_n(rst_n),

  .pub_enable(en & ros2pub_en),
  .sub_enable(en ? ros2sub_en : 4'd0),

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

  .conf_fragment_expiration(ros2_fragment_expiration),
  .conf_guid_prefix_00(ros2_guid_prefix[7:0]), .conf_guid_prefix_01(ros2_guid_prefix[15:8]),
  .conf_guid_prefix_02(ros2_guid_prefix[23:16]), .conf_guid_prefix_03(ros2_guid_prefix[31:24]),
  .conf_guid_prefix_04(ros2_guid_prefix[39:32]), .conf_guid_prefix_05(ros2_guid_prefix[47:40]),
  .conf_guid_prefix_06(ros2_guid_prefix[55:48]), .conf_guid_prefix_07(ros2_guid_prefix[63:56]),
  .conf_guid_prefix_08(ros2_guid_prefix[71:64]), .conf_guid_prefix_09(ros2_guid_prefix[79:72]),
  .conf_guid_prefix_10(ros2_guid_prefix[87:80]), .conf_guid_prefix_11(ros2_guid_prefix[95:88]),
  .conf_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
  .conf_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),
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

  .conf_sub_topic_name_0_00(ros2_sub_topic_name_0[7:0]), .conf_sub_topic_name_0_01(ros2_sub_topic_name_0[15:8]),
  .conf_sub_topic_name_0_02(ros2_sub_topic_name_0[23:16]), .conf_sub_topic_name_0_03(ros2_sub_topic_name_0[31:24]),
  .conf_sub_topic_name_0_04(ros2_sub_topic_name_0[39:32]), .conf_sub_topic_name_0_05(ros2_sub_topic_name_0[47:40]),
  .conf_sub_topic_name_0_06(ros2_sub_topic_name_0[55:48]), .conf_sub_topic_name_0_07(ros2_sub_topic_name_0[63:56]),
  .conf_sub_topic_name_0_08(ros2_sub_topic_name_0[71:64]), .conf_sub_topic_name_0_09(ros2_sub_topic_name_0[79:72]),
  .conf_sub_topic_name_0_10(ros2_sub_topic_name_0[87:80]), .conf_sub_topic_name_0_11(ros2_sub_topic_name_0[95:88]),
  .conf_sub_topic_name_0_12(ros2_sub_topic_name_0[103:96]), .conf_sub_topic_name_0_13(ros2_sub_topic_name_0[111:104]),
  .conf_sub_topic_name_0_14(ros2_sub_topic_name_0[119:112]), .conf_sub_topic_name_0_15(ros2_sub_topic_name_0[127:120]),
  .conf_sub_topic_name_0_16(ros2_sub_topic_name_0[135:128]), .conf_sub_topic_name_0_17(ros2_sub_topic_name_0[143:136]),
  .conf_sub_topic_name_0_18(ros2_sub_topic_name_0[151:144]), .conf_sub_topic_name_0_19(ros2_sub_topic_name_0[159:152]),
  .conf_sub_topic_name_0_20(ros2_sub_topic_name_0[167:160]), .conf_sub_topic_name_0_21(ros2_sub_topic_name_0[175:168]),
  .conf_sub_topic_name_0_22(ros2_sub_topic_name_0[183:176]), .conf_sub_topic_name_0_23(ros2_sub_topic_name_0[191:184]),
  .conf_sub_topic_name_0_24(ros2_sub_topic_name_0[199:192]), .conf_sub_topic_name_0_25(ros2_sub_topic_name_0[207:200]),
  .conf_sub_topic_name_0_26(ros2_sub_topic_name_0[215:208]), .conf_sub_topic_name_0_27(ros2_sub_topic_name_0[223:216]),
  .conf_sub_topic_name_0_28(ros2_sub_topic_name_0[231:224]), .conf_sub_topic_name_0_29(ros2_sub_topic_name_0[239:232]),
  .conf_sub_topic_name_0_30(ros2_sub_topic_name_0[247:240]), .conf_sub_topic_name_0_31(ros2_sub_topic_name_0[255:248]),
  .conf_sub_topic_name_len_0(ros2_sub_topic_name_len_0),
  .conf_sub_topic_type_name_0_00(ros2_sub_topic_type_name_0[7:0]), .conf_sub_topic_type_name_0_01(ros2_sub_topic_type_name_0[15:8]),
  .conf_sub_topic_type_name_0_02(ros2_sub_topic_type_name_0[23:16]), .conf_sub_topic_type_name_0_03(ros2_sub_topic_type_name_0[31:24]),
  .conf_sub_topic_type_name_0_04(ros2_sub_topic_type_name_0[39:32]), .conf_sub_topic_type_name_0_05(ros2_sub_topic_type_name_0[47:40]),
  .conf_sub_topic_type_name_0_06(ros2_sub_topic_type_name_0[55:48]), .conf_sub_topic_type_name_0_07(ros2_sub_topic_type_name_0[63:56]),
  .conf_sub_topic_type_name_0_08(ros2_sub_topic_type_name_0[71:64]), .conf_sub_topic_type_name_0_09(ros2_sub_topic_type_name_0[79:72]),
  .conf_sub_topic_type_name_0_10(ros2_sub_topic_type_name_0[87:80]), .conf_sub_topic_type_name_0_11(ros2_sub_topic_type_name_0[95:88]),
  .conf_sub_topic_type_name_0_12(ros2_sub_topic_type_name_0[103:96]), .conf_sub_topic_type_name_0_13(ros2_sub_topic_type_name_0[111:104]),
  .conf_sub_topic_type_name_0_14(ros2_sub_topic_type_name_0[119:112]), .conf_sub_topic_type_name_0_15(ros2_sub_topic_type_name_0[127:120]),
  .conf_sub_topic_type_name_0_16(ros2_sub_topic_type_name_0[135:128]), .conf_sub_topic_type_name_0_17(ros2_sub_topic_type_name_0[143:136]),
  .conf_sub_topic_type_name_0_18(ros2_sub_topic_type_name_0[151:144]), .conf_sub_topic_type_name_0_19(ros2_sub_topic_type_name_0[159:152]),
  .conf_sub_topic_type_name_0_20(ros2_sub_topic_type_name_0[167:160]), .conf_sub_topic_type_name_0_21(ros2_sub_topic_type_name_0[175:168]),
  .conf_sub_topic_type_name_0_22(ros2_sub_topic_type_name_0[183:176]), .conf_sub_topic_type_name_0_23(ros2_sub_topic_type_name_0[191:184]),
  .conf_sub_topic_type_name_0_24(ros2_sub_topic_type_name_0[199:192]), .conf_sub_topic_type_name_0_25(ros2_sub_topic_type_name_0[207:200]),
  .conf_sub_topic_type_name_0_26(ros2_sub_topic_type_name_0[215:208]), .conf_sub_topic_type_name_0_27(ros2_sub_topic_type_name_0[223:216]),
  .conf_sub_topic_type_name_0_28(ros2_sub_topic_type_name_0[231:224]), .conf_sub_topic_type_name_0_29(ros2_sub_topic_type_name_0[239:232]),
  .conf_sub_topic_type_name_0_30(ros2_sub_topic_type_name_0[247:240]), .conf_sub_topic_type_name_0_31(ros2_sub_topic_type_name_0[255:248]),
  .conf_sub_topic_type_name_0_32(ros2_sub_topic_type_name_0[263:256]), .conf_sub_topic_type_name_0_33(ros2_sub_topic_type_name_0[271:264]),
  .conf_sub_topic_type_name_0_34(ros2_sub_topic_type_name_0[279:272]), .conf_sub_topic_type_name_0_35(ros2_sub_topic_type_name_0[287:280]),
  .conf_sub_topic_type_name_0_36(ros2_sub_topic_type_name_0[295:288]), .conf_sub_topic_type_name_0_37(ros2_sub_topic_type_name_0[303:296]),
  .conf_sub_topic_type_name_0_38(ros2_sub_topic_type_name_0[311:304]), .conf_sub_topic_type_name_0_39(ros2_sub_topic_type_name_0[319:312]),
  .conf_sub_topic_type_name_0_40(ros2_sub_topic_type_name_0[327:320]), .conf_sub_topic_type_name_0_41(ros2_sub_topic_type_name_0[335:328]),
  .conf_sub_topic_type_name_0_42(ros2_sub_topic_type_name_0[343:336]), .conf_sub_topic_type_name_0_43(ros2_sub_topic_type_name_0[351:344]),
  .conf_sub_topic_type_name_0_44(ros2_sub_topic_type_name_0[359:352]), .conf_sub_topic_type_name_0_45(ros2_sub_topic_type_name_0[367:360]),
  .conf_sub_topic_type_name_0_46(ros2_sub_topic_type_name_0[375:368]), .conf_sub_topic_type_name_0_47(ros2_sub_topic_type_name_0[383:376]),
  .conf_sub_topic_type_name_0_48(ros2_sub_topic_type_name_0[391:384]), .conf_sub_topic_type_name_0_49(ros2_sub_topic_type_name_0[399:392]),
  .conf_sub_topic_type_name_0_50(ros2_sub_topic_type_name_0[407:400]), .conf_sub_topic_type_name_0_51(ros2_sub_topic_type_name_0[415:408]),
  .conf_sub_topic_type_name_0_52(ros2_sub_topic_type_name_0[423:416]), .conf_sub_topic_type_name_0_53(ros2_sub_topic_type_name_0[431:424]),
  .conf_sub_topic_type_name_0_54(ros2_sub_topic_type_name_0[439:432]), .conf_sub_topic_type_name_0_55(ros2_sub_topic_type_name_0[447:440]),
  .conf_sub_topic_type_name_0_56(ros2_sub_topic_type_name_0[455:448]), .conf_sub_topic_type_name_0_57(ros2_sub_topic_type_name_0[463:456]),
  .conf_sub_topic_type_name_0_58(ros2_sub_topic_type_name_0[471:464]), .conf_sub_topic_type_name_0_59(ros2_sub_topic_type_name_0[479:472]),
  .conf_sub_topic_type_name_0_60(ros2_sub_topic_type_name_0[487:480]), .conf_sub_topic_type_name_0_61(ros2_sub_topic_type_name_0[495:488]),
  .conf_sub_topic_type_name_0_62(ros2_sub_topic_type_name_0[503:496]), .conf_sub_topic_type_name_0_63(ros2_sub_topic_type_name_0[511:504]),
  .conf_sub_topic_type_name_len_0(ros2_sub_topic_type_name_len_0),

  .conf_sub_topic_name_1_00(ros2_sub_topic_name_1[7:0]), .conf_sub_topic_name_1_01(ros2_sub_topic_name_1[15:8]),
  .conf_sub_topic_name_1_02(ros2_sub_topic_name_1[23:16]), .conf_sub_topic_name_1_03(ros2_sub_topic_name_1[31:24]),
  .conf_sub_topic_name_1_04(ros2_sub_topic_name_1[39:32]), .conf_sub_topic_name_1_05(ros2_sub_topic_name_1[47:40]),
  .conf_sub_topic_name_1_06(ros2_sub_topic_name_1[55:48]), .conf_sub_topic_name_1_07(ros2_sub_topic_name_1[63:56]),
  .conf_sub_topic_name_1_08(ros2_sub_topic_name_1[71:64]), .conf_sub_topic_name_1_09(ros2_sub_topic_name_1[79:72]),
  .conf_sub_topic_name_1_10(ros2_sub_topic_name_1[87:80]), .conf_sub_topic_name_1_11(ros2_sub_topic_name_1[95:88]),
  .conf_sub_topic_name_1_12(ros2_sub_topic_name_1[103:96]), .conf_sub_topic_name_1_13(ros2_sub_topic_name_1[111:104]),
  .conf_sub_topic_name_1_14(ros2_sub_topic_name_1[119:112]), .conf_sub_topic_name_1_15(ros2_sub_topic_name_1[127:120]),
  .conf_sub_topic_name_1_16(ros2_sub_topic_name_1[135:128]), .conf_sub_topic_name_1_17(ros2_sub_topic_name_1[143:136]),
  .conf_sub_topic_name_1_18(ros2_sub_topic_name_1[151:144]), .conf_sub_topic_name_1_19(ros2_sub_topic_name_1[159:152]),
  .conf_sub_topic_name_1_20(ros2_sub_topic_name_1[167:160]), .conf_sub_topic_name_1_21(ros2_sub_topic_name_1[175:168]),
  .conf_sub_topic_name_1_22(ros2_sub_topic_name_1[183:176]), .conf_sub_topic_name_1_23(ros2_sub_topic_name_1[191:184]),
  .conf_sub_topic_name_1_24(ros2_sub_topic_name_1[199:192]), .conf_sub_topic_name_1_25(ros2_sub_topic_name_1[207:200]),
  .conf_sub_topic_name_1_26(ros2_sub_topic_name_1[215:208]), .conf_sub_topic_name_1_27(ros2_sub_topic_name_1[223:216]),
  .conf_sub_topic_name_1_28(ros2_sub_topic_name_1[231:224]), .conf_sub_topic_name_1_29(ros2_sub_topic_name_1[239:232]),
  .conf_sub_topic_name_1_30(ros2_sub_topic_name_1[247:240]), .conf_sub_topic_name_1_31(ros2_sub_topic_name_1[255:248]),
  .conf_sub_topic_name_len_1(ros2_sub_topic_name_len_1),
  .conf_sub_topic_type_name_1_00(ros2_sub_topic_type_name_1[7:0]), .conf_sub_topic_type_name_1_01(ros2_sub_topic_type_name_1[15:8]),
  .conf_sub_topic_type_name_1_02(ros2_sub_topic_type_name_1[23:16]), .conf_sub_topic_type_name_1_03(ros2_sub_topic_type_name_1[31:24]),
  .conf_sub_topic_type_name_1_04(ros2_sub_topic_type_name_1[39:32]), .conf_sub_topic_type_name_1_05(ros2_sub_topic_type_name_1[47:40]),
  .conf_sub_topic_type_name_1_06(ros2_sub_topic_type_name_1[55:48]), .conf_sub_topic_type_name_1_07(ros2_sub_topic_type_name_1[63:56]),
  .conf_sub_topic_type_name_1_08(ros2_sub_topic_type_name_1[71:64]), .conf_sub_topic_type_name_1_09(ros2_sub_topic_type_name_1[79:72]),
  .conf_sub_topic_type_name_1_10(ros2_sub_topic_type_name_1[87:80]), .conf_sub_topic_type_name_1_11(ros2_sub_topic_type_name_1[95:88]),
  .conf_sub_topic_type_name_1_12(ros2_sub_topic_type_name_1[103:96]), .conf_sub_topic_type_name_1_13(ros2_sub_topic_type_name_1[111:104]),
  .conf_sub_topic_type_name_1_14(ros2_sub_topic_type_name_1[119:112]), .conf_sub_topic_type_name_1_15(ros2_sub_topic_type_name_1[127:120]),
  .conf_sub_topic_type_name_1_16(ros2_sub_topic_type_name_1[135:128]), .conf_sub_topic_type_name_1_17(ros2_sub_topic_type_name_1[143:136]),
  .conf_sub_topic_type_name_1_18(ros2_sub_topic_type_name_1[151:144]), .conf_sub_topic_type_name_1_19(ros2_sub_topic_type_name_1[159:152]),
  .conf_sub_topic_type_name_1_20(ros2_sub_topic_type_name_1[167:160]), .conf_sub_topic_type_name_1_21(ros2_sub_topic_type_name_1[175:168]),
  .conf_sub_topic_type_name_1_22(ros2_sub_topic_type_name_1[183:176]), .conf_sub_topic_type_name_1_23(ros2_sub_topic_type_name_1[191:184]),
  .conf_sub_topic_type_name_1_24(ros2_sub_topic_type_name_1[199:192]), .conf_sub_topic_type_name_1_25(ros2_sub_topic_type_name_1[207:200]),
  .conf_sub_topic_type_name_1_26(ros2_sub_topic_type_name_1[215:208]), .conf_sub_topic_type_name_1_27(ros2_sub_topic_type_name_1[223:216]),
  .conf_sub_topic_type_name_1_28(ros2_sub_topic_type_name_1[231:224]), .conf_sub_topic_type_name_1_29(ros2_sub_topic_type_name_1[239:232]),
  .conf_sub_topic_type_name_1_30(ros2_sub_topic_type_name_1[247:240]), .conf_sub_topic_type_name_1_31(ros2_sub_topic_type_name_1[255:248]),
  .conf_sub_topic_type_name_1_32(ros2_sub_topic_type_name_1[263:256]), .conf_sub_topic_type_name_1_33(ros2_sub_topic_type_name_1[271:264]),
  .conf_sub_topic_type_name_1_34(ros2_sub_topic_type_name_1[279:272]), .conf_sub_topic_type_name_1_35(ros2_sub_topic_type_name_1[287:280]),
  .conf_sub_topic_type_name_1_36(ros2_sub_topic_type_name_1[295:288]), .conf_sub_topic_type_name_1_37(ros2_sub_topic_type_name_1[303:296]),
  .conf_sub_topic_type_name_1_38(ros2_sub_topic_type_name_1[311:304]), .conf_sub_topic_type_name_1_39(ros2_sub_topic_type_name_1[319:312]),
  .conf_sub_topic_type_name_1_40(ros2_sub_topic_type_name_1[327:320]), .conf_sub_topic_type_name_1_41(ros2_sub_topic_type_name_1[335:328]),
  .conf_sub_topic_type_name_1_42(ros2_sub_topic_type_name_1[343:336]), .conf_sub_topic_type_name_1_43(ros2_sub_topic_type_name_1[351:344]),
  .conf_sub_topic_type_name_1_44(ros2_sub_topic_type_name_1[359:352]), .conf_sub_topic_type_name_1_45(ros2_sub_topic_type_name_1[367:360]),
  .conf_sub_topic_type_name_1_46(ros2_sub_topic_type_name_1[375:368]), .conf_sub_topic_type_name_1_47(ros2_sub_topic_type_name_1[383:376]),
  .conf_sub_topic_type_name_1_48(ros2_sub_topic_type_name_1[391:384]), .conf_sub_topic_type_name_1_49(ros2_sub_topic_type_name_1[399:392]),
  .conf_sub_topic_type_name_1_50(ros2_sub_topic_type_name_1[407:400]), .conf_sub_topic_type_name_1_51(ros2_sub_topic_type_name_1[415:408]),
  .conf_sub_topic_type_name_1_52(ros2_sub_topic_type_name_1[423:416]), .conf_sub_topic_type_name_1_53(ros2_sub_topic_type_name_1[431:424]),
  .conf_sub_topic_type_name_1_54(ros2_sub_topic_type_name_1[439:432]), .conf_sub_topic_type_name_1_55(ros2_sub_topic_type_name_1[447:440]),
  .conf_sub_topic_type_name_1_56(ros2_sub_topic_type_name_1[455:448]), .conf_sub_topic_type_name_1_57(ros2_sub_topic_type_name_1[463:456]),
  .conf_sub_topic_type_name_1_58(ros2_sub_topic_type_name_1[471:464]), .conf_sub_topic_type_name_1_59(ros2_sub_topic_type_name_1[479:472]),
  .conf_sub_topic_type_name_1_60(ros2_sub_topic_type_name_1[487:480]), .conf_sub_topic_type_name_1_61(ros2_sub_topic_type_name_1[495:488]),
  .conf_sub_topic_type_name_1_62(ros2_sub_topic_type_name_1[503:496]), .conf_sub_topic_type_name_1_63(ros2_sub_topic_type_name_1[511:504]),
  .conf_sub_topic_type_name_len_1(ros2_sub_topic_type_name_len_1),

  .conf_sub_topic_name_2_00(ros2_sub_topic_name_2[7:0]), .conf_sub_topic_name_2_01(ros2_sub_topic_name_2[15:8]),
  .conf_sub_topic_name_2_02(ros2_sub_topic_name_2[23:16]), .conf_sub_topic_name_2_03(ros2_sub_topic_name_2[31:24]),
  .conf_sub_topic_name_2_04(ros2_sub_topic_name_2[39:32]), .conf_sub_topic_name_2_05(ros2_sub_topic_name_2[47:40]),
  .conf_sub_topic_name_2_06(ros2_sub_topic_name_2[55:48]), .conf_sub_topic_name_2_07(ros2_sub_topic_name_2[63:56]),
  .conf_sub_topic_name_2_08(ros2_sub_topic_name_2[71:64]), .conf_sub_topic_name_2_09(ros2_sub_topic_name_2[79:72]),
  .conf_sub_topic_name_2_10(ros2_sub_topic_name_2[87:80]), .conf_sub_topic_name_2_11(ros2_sub_topic_name_2[95:88]),
  .conf_sub_topic_name_2_12(ros2_sub_topic_name_2[103:96]), .conf_sub_topic_name_2_13(ros2_sub_topic_name_2[111:104]),
  .conf_sub_topic_name_2_14(ros2_sub_topic_name_2[119:112]), .conf_sub_topic_name_2_15(ros2_sub_topic_name_2[127:120]),
  .conf_sub_topic_name_2_16(ros2_sub_topic_name_2[135:128]), .conf_sub_topic_name_2_17(ros2_sub_topic_name_2[143:136]),
  .conf_sub_topic_name_2_18(ros2_sub_topic_name_2[151:144]), .conf_sub_topic_name_2_19(ros2_sub_topic_name_2[159:152]),
  .conf_sub_topic_name_2_20(ros2_sub_topic_name_2[167:160]), .conf_sub_topic_name_2_21(ros2_sub_topic_name_2[175:168]),
  .conf_sub_topic_name_2_22(ros2_sub_topic_name_2[183:176]), .conf_sub_topic_name_2_23(ros2_sub_topic_name_2[191:184]),
  .conf_sub_topic_name_2_24(ros2_sub_topic_name_2[199:192]), .conf_sub_topic_name_2_25(ros2_sub_topic_name_2[207:200]),
  .conf_sub_topic_name_2_26(ros2_sub_topic_name_2[215:208]), .conf_sub_topic_name_2_27(ros2_sub_topic_name_2[223:216]),
  .conf_sub_topic_name_2_28(ros2_sub_topic_name_2[231:224]), .conf_sub_topic_name_2_29(ros2_sub_topic_name_2[239:232]),
  .conf_sub_topic_name_2_30(ros2_sub_topic_name_2[247:240]), .conf_sub_topic_name_2_31(ros2_sub_topic_name_2[255:248]),
  .conf_sub_topic_name_len_2(ros2_sub_topic_name_len_2),
  .conf_sub_topic_type_name_2_00(ros2_sub_topic_type_name_2[7:0]), .conf_sub_topic_type_name_2_01(ros2_sub_topic_type_name_2[15:8]),
  .conf_sub_topic_type_name_2_02(ros2_sub_topic_type_name_2[23:16]), .conf_sub_topic_type_name_2_03(ros2_sub_topic_type_name_2[31:24]),
  .conf_sub_topic_type_name_2_04(ros2_sub_topic_type_name_2[39:32]), .conf_sub_topic_type_name_2_05(ros2_sub_topic_type_name_2[47:40]),
  .conf_sub_topic_type_name_2_06(ros2_sub_topic_type_name_2[55:48]), .conf_sub_topic_type_name_2_07(ros2_sub_topic_type_name_2[63:56]),
  .conf_sub_topic_type_name_2_08(ros2_sub_topic_type_name_2[71:64]), .conf_sub_topic_type_name_2_09(ros2_sub_topic_type_name_2[79:72]),
  .conf_sub_topic_type_name_2_10(ros2_sub_topic_type_name_2[87:80]), .conf_sub_topic_type_name_2_11(ros2_sub_topic_type_name_2[95:88]),
  .conf_sub_topic_type_name_2_12(ros2_sub_topic_type_name_2[103:96]), .conf_sub_topic_type_name_2_13(ros2_sub_topic_type_name_2[111:104]),
  .conf_sub_topic_type_name_2_14(ros2_sub_topic_type_name_2[119:112]), .conf_sub_topic_type_name_2_15(ros2_sub_topic_type_name_2[127:120]),
  .conf_sub_topic_type_name_2_16(ros2_sub_topic_type_name_2[135:128]), .conf_sub_topic_type_name_2_17(ros2_sub_topic_type_name_2[143:136]),
  .conf_sub_topic_type_name_2_18(ros2_sub_topic_type_name_2[151:144]), .conf_sub_topic_type_name_2_19(ros2_sub_topic_type_name_2[159:152]),
  .conf_sub_topic_type_name_2_20(ros2_sub_topic_type_name_2[167:160]), .conf_sub_topic_type_name_2_21(ros2_sub_topic_type_name_2[175:168]),
  .conf_sub_topic_type_name_2_22(ros2_sub_topic_type_name_2[183:176]), .conf_sub_topic_type_name_2_23(ros2_sub_topic_type_name_2[191:184]),
  .conf_sub_topic_type_name_2_24(ros2_sub_topic_type_name_2[199:192]), .conf_sub_topic_type_name_2_25(ros2_sub_topic_type_name_2[207:200]),
  .conf_sub_topic_type_name_2_26(ros2_sub_topic_type_name_2[215:208]), .conf_sub_topic_type_name_2_27(ros2_sub_topic_type_name_2[223:216]),
  .conf_sub_topic_type_name_2_28(ros2_sub_topic_type_name_2[231:224]), .conf_sub_topic_type_name_2_29(ros2_sub_topic_type_name_2[239:232]),
  .conf_sub_topic_type_name_2_30(ros2_sub_topic_type_name_2[247:240]), .conf_sub_topic_type_name_2_31(ros2_sub_topic_type_name_2[255:248]),
  .conf_sub_topic_type_name_2_32(ros2_sub_topic_type_name_2[263:256]), .conf_sub_topic_type_name_2_33(ros2_sub_topic_type_name_2[271:264]),
  .conf_sub_topic_type_name_2_34(ros2_sub_topic_type_name_2[279:272]), .conf_sub_topic_type_name_2_35(ros2_sub_topic_type_name_2[287:280]),
  .conf_sub_topic_type_name_2_36(ros2_sub_topic_type_name_2[295:288]), .conf_sub_topic_type_name_2_37(ros2_sub_topic_type_name_2[303:296]),
  .conf_sub_topic_type_name_2_38(ros2_sub_topic_type_name_2[311:304]), .conf_sub_topic_type_name_2_39(ros2_sub_topic_type_name_2[319:312]),
  .conf_sub_topic_type_name_2_40(ros2_sub_topic_type_name_2[327:320]), .conf_sub_topic_type_name_2_41(ros2_sub_topic_type_name_2[335:328]),
  .conf_sub_topic_type_name_2_42(ros2_sub_topic_type_name_2[343:336]), .conf_sub_topic_type_name_2_43(ros2_sub_topic_type_name_2[351:344]),
  .conf_sub_topic_type_name_2_44(ros2_sub_topic_type_name_2[359:352]), .conf_sub_topic_type_name_2_45(ros2_sub_topic_type_name_2[367:360]),
  .conf_sub_topic_type_name_2_46(ros2_sub_topic_type_name_2[375:368]), .conf_sub_topic_type_name_2_47(ros2_sub_topic_type_name_2[383:376]),
  .conf_sub_topic_type_name_2_48(ros2_sub_topic_type_name_2[391:384]), .conf_sub_topic_type_name_2_49(ros2_sub_topic_type_name_2[399:392]),
  .conf_sub_topic_type_name_2_50(ros2_sub_topic_type_name_2[407:400]), .conf_sub_topic_type_name_2_51(ros2_sub_topic_type_name_2[415:408]),
  .conf_sub_topic_type_name_2_52(ros2_sub_topic_type_name_2[423:416]), .conf_sub_topic_type_name_2_53(ros2_sub_topic_type_name_2[431:424]),
  .conf_sub_topic_type_name_2_54(ros2_sub_topic_type_name_2[439:432]), .conf_sub_topic_type_name_2_55(ros2_sub_topic_type_name_2[447:440]),
  .conf_sub_topic_type_name_2_56(ros2_sub_topic_type_name_2[455:448]), .conf_sub_topic_type_name_2_57(ros2_sub_topic_type_name_2[463:456]),
  .conf_sub_topic_type_name_2_58(ros2_sub_topic_type_name_2[471:464]), .conf_sub_topic_type_name_2_59(ros2_sub_topic_type_name_2[479:472]),
  .conf_sub_topic_type_name_2_60(ros2_sub_topic_type_name_2[487:480]), .conf_sub_topic_type_name_2_61(ros2_sub_topic_type_name_2[495:488]),
  .conf_sub_topic_type_name_2_62(ros2_sub_topic_type_name_2[503:496]), .conf_sub_topic_type_name_2_63(ros2_sub_topic_type_name_2[511:504]),
  .conf_sub_topic_type_name_len_2(ros2_sub_topic_type_name_len_2),

  .conf_sub_topic_name_3_00(ros2_sub_topic_name_3[7:0]), .conf_sub_topic_name_3_01(ros2_sub_topic_name_3[15:8]),
  .conf_sub_topic_name_3_02(ros2_sub_topic_name_3[23:16]), .conf_sub_topic_name_3_03(ros2_sub_topic_name_3[31:24]),
  .conf_sub_topic_name_3_04(ros2_sub_topic_name_3[39:32]), .conf_sub_topic_name_3_05(ros2_sub_topic_name_3[47:40]),
  .conf_sub_topic_name_3_06(ros2_sub_topic_name_3[55:48]), .conf_sub_topic_name_3_07(ros2_sub_topic_name_3[63:56]),
  .conf_sub_topic_name_3_08(ros2_sub_topic_name_3[71:64]), .conf_sub_topic_name_3_09(ros2_sub_topic_name_3[79:72]),
  .conf_sub_topic_name_3_10(ros2_sub_topic_name_3[87:80]), .conf_sub_topic_name_3_11(ros2_sub_topic_name_3[95:88]),
  .conf_sub_topic_name_3_12(ros2_sub_topic_name_3[103:96]), .conf_sub_topic_name_3_13(ros2_sub_topic_name_3[111:104]),
  .conf_sub_topic_name_3_14(ros2_sub_topic_name_3[119:112]), .conf_sub_topic_name_3_15(ros2_sub_topic_name_3[127:120]),
  .conf_sub_topic_name_3_16(ros2_sub_topic_name_3[135:128]), .conf_sub_topic_name_3_17(ros2_sub_topic_name_3[143:136]),
  .conf_sub_topic_name_3_18(ros2_sub_topic_name_3[151:144]), .conf_sub_topic_name_3_19(ros2_sub_topic_name_3[159:152]),
  .conf_sub_topic_name_3_20(ros2_sub_topic_name_3[167:160]), .conf_sub_topic_name_3_21(ros2_sub_topic_name_3[175:168]),
  .conf_sub_topic_name_3_22(ros2_sub_topic_name_3[183:176]), .conf_sub_topic_name_3_23(ros2_sub_topic_name_3[191:184]),
  .conf_sub_topic_name_3_24(ros2_sub_topic_name_3[199:192]), .conf_sub_topic_name_3_25(ros2_sub_topic_name_3[207:200]),
  .conf_sub_topic_name_3_26(ros2_sub_topic_name_3[215:208]), .conf_sub_topic_name_3_27(ros2_sub_topic_name_3[223:216]),
  .conf_sub_topic_name_3_28(ros2_sub_topic_name_3[231:224]), .conf_sub_topic_name_3_29(ros2_sub_topic_name_3[239:232]),
  .conf_sub_topic_name_3_30(ros2_sub_topic_name_3[247:240]), .conf_sub_topic_name_3_31(ros2_sub_topic_name_3[255:248]),
  .conf_sub_topic_name_len_3(ros2_sub_topic_name_len_3),
  .conf_sub_topic_type_name_3_00(ros2_sub_topic_type_name_3[7:0]), .conf_sub_topic_type_name_3_01(ros2_sub_topic_type_name_3[15:8]),
  .conf_sub_topic_type_name_3_02(ros2_sub_topic_type_name_3[23:16]), .conf_sub_topic_type_name_3_03(ros2_sub_topic_type_name_3[31:24]),
  .conf_sub_topic_type_name_3_04(ros2_sub_topic_type_name_3[39:32]), .conf_sub_topic_type_name_3_05(ros2_sub_topic_type_name_3[47:40]),
  .conf_sub_topic_type_name_3_06(ros2_sub_topic_type_name_3[55:48]), .conf_sub_topic_type_name_3_07(ros2_sub_topic_type_name_3[63:56]),
  .conf_sub_topic_type_name_3_08(ros2_sub_topic_type_name_3[71:64]), .conf_sub_topic_type_name_3_09(ros2_sub_topic_type_name_3[79:72]),
  .conf_sub_topic_type_name_3_10(ros2_sub_topic_type_name_3[87:80]), .conf_sub_topic_type_name_3_11(ros2_sub_topic_type_name_3[95:88]),
  .conf_sub_topic_type_name_3_12(ros2_sub_topic_type_name_3[103:96]), .conf_sub_topic_type_name_3_13(ros2_sub_topic_type_name_3[111:104]),
  .conf_sub_topic_type_name_3_14(ros2_sub_topic_type_name_3[119:112]), .conf_sub_topic_type_name_3_15(ros2_sub_topic_type_name_3[127:120]),
  .conf_sub_topic_type_name_3_16(ros2_sub_topic_type_name_3[135:128]), .conf_sub_topic_type_name_3_17(ros2_sub_topic_type_name_3[143:136]),
  .conf_sub_topic_type_name_3_18(ros2_sub_topic_type_name_3[151:144]), .conf_sub_topic_type_name_3_19(ros2_sub_topic_type_name_3[159:152]),
  .conf_sub_topic_type_name_3_20(ros2_sub_topic_type_name_3[167:160]), .conf_sub_topic_type_name_3_21(ros2_sub_topic_type_name_3[175:168]),
  .conf_sub_topic_type_name_3_22(ros2_sub_topic_type_name_3[183:176]), .conf_sub_topic_type_name_3_23(ros2_sub_topic_type_name_3[191:184]),
  .conf_sub_topic_type_name_3_24(ros2_sub_topic_type_name_3[199:192]), .conf_sub_topic_type_name_3_25(ros2_sub_topic_type_name_3[207:200]),
  .conf_sub_topic_type_name_3_26(ros2_sub_topic_type_name_3[215:208]), .conf_sub_topic_type_name_3_27(ros2_sub_topic_type_name_3[223:216]),
  .conf_sub_topic_type_name_3_28(ros2_sub_topic_type_name_3[231:224]), .conf_sub_topic_type_name_3_29(ros2_sub_topic_type_name_3[239:232]),
  .conf_sub_topic_type_name_3_30(ros2_sub_topic_type_name_3[247:240]), .conf_sub_topic_type_name_3_31(ros2_sub_topic_type_name_3[255:248]),
  .conf_sub_topic_type_name_3_32(ros2_sub_topic_type_name_3[263:256]), .conf_sub_topic_type_name_3_33(ros2_sub_topic_type_name_3[271:264]),
  .conf_sub_topic_type_name_3_34(ros2_sub_topic_type_name_3[279:272]), .conf_sub_topic_type_name_3_35(ros2_sub_topic_type_name_3[287:280]),
  .conf_sub_topic_type_name_3_36(ros2_sub_topic_type_name_3[295:288]), .conf_sub_topic_type_name_3_37(ros2_sub_topic_type_name_3[303:296]),
  .conf_sub_topic_type_name_3_38(ros2_sub_topic_type_name_3[311:304]), .conf_sub_topic_type_name_3_39(ros2_sub_topic_type_name_3[319:312]),
  .conf_sub_topic_type_name_3_40(ros2_sub_topic_type_name_3[327:320]), .conf_sub_topic_type_name_3_41(ros2_sub_topic_type_name_3[335:328]),
  .conf_sub_topic_type_name_3_42(ros2_sub_topic_type_name_3[343:336]), .conf_sub_topic_type_name_3_43(ros2_sub_topic_type_name_3[351:344]),
  .conf_sub_topic_type_name_3_44(ros2_sub_topic_type_name_3[359:352]), .conf_sub_topic_type_name_3_45(ros2_sub_topic_type_name_3[367:360]),
  .conf_sub_topic_type_name_3_46(ros2_sub_topic_type_name_3[375:368]), .conf_sub_topic_type_name_3_47(ros2_sub_topic_type_name_3[383:376]),
  .conf_sub_topic_type_name_3_48(ros2_sub_topic_type_name_3[391:384]), .conf_sub_topic_type_name_3_49(ros2_sub_topic_type_name_3[399:392]),
  .conf_sub_topic_type_name_3_50(ros2_sub_topic_type_name_3[407:400]), .conf_sub_topic_type_name_3_51(ros2_sub_topic_type_name_3[415:408]),
  .conf_sub_topic_type_name_3_52(ros2_sub_topic_type_name_3[423:416]), .conf_sub_topic_type_name_3_53(ros2_sub_topic_type_name_3[431:424]),
  .conf_sub_topic_type_name_3_54(ros2_sub_topic_type_name_3[439:432]), .conf_sub_topic_type_name_3_55(ros2_sub_topic_type_name_3[447:440]),
  .conf_sub_topic_type_name_3_56(ros2_sub_topic_type_name_3[455:448]), .conf_sub_topic_type_name_3_57(ros2_sub_topic_type_name_3[463:456]),
  .conf_sub_topic_type_name_3_58(ros2_sub_topic_type_name_3[471:464]), .conf_sub_topic_type_name_3_59(ros2_sub_topic_type_name_3[479:472]),
  .conf_sub_topic_type_name_3_60(ros2_sub_topic_type_name_3[487:480]), .conf_sub_topic_type_name_3_61(ros2_sub_topic_type_name_3[495:488]),
  .conf_sub_topic_type_name_3_62(ros2_sub_topic_type_name_3[503:496]), .conf_sub_topic_type_name_3_63(ros2_sub_topic_type_name_3[511:504]),
  .conf_sub_topic_type_name_len_3(ros2_sub_topic_type_name_len_3),

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
  .sub_app_data_recv_we(sub_app_data_recv_ap_vld),
  .sub_app_data_recv_wd(sub_app_data_recv),
  .sub_app_data_req_we(ros2_sub_app_data_ip_req),
  .sub_app_data_req_wd(),
  .sub_app_data_rel_we(ros2_sub_app_data_ip_rel),
  .sub_app_data_rel_wd(),
  .sub_app_data_grant_rd({7'b0, ros2_sub_app_data_ip_grant}),

  .cnt_interval_set_wd(),
  .cnt_interval_set_we(ros2_cnt_interval_set),
  .cnt_spdp_wr_set_wd(),
  .cnt_spdp_wr_set_we(ros2_cnt_spdp_wr_set),
  .cnt_sedp_pub_wr_set_wd(),
  .cnt_sedp_pub_wr_set_we(ros2_cnt_sedp_pub_wr_set),
  .cnt_sedp_sub_wr_set_wd(),
  .cnt_sedp_sub_wr_set_we(ros2_cnt_sedp_sub_wr_set),
  .cnt_sedp_pub_hb_set_wd(),
  .cnt_sedp_pub_hb_set_we(ros2_cnt_sedp_pub_hb_set),
  .cnt_sedp_sub_hb_set_wd(),
  .cnt_sedp_sub_hb_set_we(ros2_cnt_sedp_sub_hb_set),
  .cnt_sedp_pub_an_set_wd(),
  .cnt_sedp_pub_an_set_we(ros2_cnt_sedp_pub_an_set),
  .cnt_sedp_sub_an_set_wd(),
  .cnt_sedp_sub_an_set_we(ros2_cnt_sedp_sub_an_set),
  .cnt_app_wr_set_wd(),
  .cnt_app_wr_set_we(ros2_cnt_app_wr_set),

  .cnt_interval_elapsed(ros2_cnt_interval_elapsed),
  .cnt_spdp_wr_elapsed(ros2_cnt_spdp_wr_elapsed),
  .cnt_sedp_pub_wr_elapsed(ros2_cnt_sedp_pub_wr_elapsed),
  .cnt_sedp_sub_wr_elapsed(ros2_cnt_sedp_sub_wr_elapsed),
  .cnt_sedp_pub_hb_elapsed(ros2_cnt_sedp_pub_hb_elapsed),
  .cnt_sedp_sub_hb_elapsed(ros2_cnt_sedp_sub_hb_elapsed),
  .cnt_sedp_pub_an_elapsed(ros2_cnt_sedp_pub_an_elapsed),
  .cnt_sedp_sub_an_elapsed(ros2_cnt_sedp_sub_an_elapsed),
  .cnt_app_wr_elapsed(ros2_cnt_app_wr_elapsed),

  .udp_rxbuf_rel_we(udp_rxbuf_ip_rel),
  .udp_rxbuf_rel_wd(),
  .udp_rxbuf_grant_rd({7'b0, udp_rxbuf_ip_grant}),

  .udp_txbuf_rel_we(udp_txbuf_ip_rel),
  .udp_txbuf_rel_wd(),
  .udp_txbuf_grant_rd({7'b0, udp_txbuf_ip_grant}),

  .timestamp_i64(local_timestamp),

  .xout_i(9'h0),
  .xout_o()
);
`endif

endmodule

module ros2rapper_tx_counters #
(
    parameter PRESCALER_DIV               = 64,
    parameter TX_INTERVAL_COUNT           = (100_000_000 / PRESCALER_DIV) / 100,
    parameter TX_PERIOD_SPDP_WR_COUNT     = (100_000_000 / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_PUB_WR_COUNT = (100_000_000 / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_SUB_WR_COUNT = (100_000_000 / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_PUB_HB_COUNT = (100_000_000 / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_SUB_HB_COUNT = (100_000_000 / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_PUB_AN_COUNT = (100_000_000 / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_SEDP_SUB_AN_COUNT = (100_000_000 / PRESCALER_DIV) * 3,
    parameter TX_PERIOD_APP_WR_COUNT      = (100_000_000 / PRESCALER_DIV) * 3
)
(
    input wire i_clk,
    input wire i_rst_n,

    input wire i_cnt_interval_set,
    input wire i_cnt_spdp_wr_set,
    input wire i_cnt_sedp_pub_wr_set,
    input wire i_cnt_sedp_sub_wr_set,
    input wire i_cnt_sedp_pub_hb_set,
    input wire i_cnt_sedp_sub_hb_set,
    input wire i_cnt_sedp_pub_an_set,
    input wire i_cnt_sedp_sub_an_set,
    input wire i_cnt_app_wr_set,

    output wire o_cnt_interval_elapsed,
    output wire o_cnt_spdp_wr_elapsed,
    output wire o_cnt_sedp_pub_wr_elapsed,
    output wire o_cnt_sedp_sub_wr_elapsed,
    output wire o_cnt_sedp_pub_hb_elapsed,
    output wire o_cnt_sedp_sub_hb_elapsed,
    output wire o_cnt_sedp_pub_an_elapsed,
    output wire o_cnt_sedp_sub_an_elapsed,
    output wire o_cnt_app_wr_elapsed
);
    // Counters for ROS2rapper TX scheduler
    reg [$clog2(PRESCALER_DIV                )-1:0] cnt_prescaler;
    reg [$clog2(TX_INTERVAL_COUNT          +1)-1:0] cnt_interval;
    reg [$clog2(TX_PERIOD_SPDP_WR_COUNT    +1)-1:0] cnt_spdp_wr;
    reg [$clog2(TX_PERIOD_SEDP_PUB_WR_COUNT+1)-1:0] cnt_sedp_pub_wr;
    reg [$clog2(TX_PERIOD_SEDP_SUB_WR_COUNT+1)-1:0] cnt_sedp_sub_wr;
    reg [$clog2(TX_PERIOD_SEDP_PUB_HB_COUNT+1)-1:0] cnt_sedp_pub_hb;
    reg [$clog2(TX_PERIOD_SEDP_SUB_HB_COUNT+1)-1:0] cnt_sedp_sub_hb;
    reg [$clog2(TX_PERIOD_SEDP_PUB_AN_COUNT+1)-1:0] cnt_sedp_pub_an;
    reg [$clog2(TX_PERIOD_SEDP_SUB_AN_COUNT+1)-1:0] cnt_sedp_sub_an;
    reg [$clog2(TX_PERIOD_APP_WR_COUNT     +1)-1:0] cnt_app_wr;

    assign o_cnt_interval_elapsed    = (cnt_interval == 0);
    assign o_cnt_spdp_wr_elapsed     = (cnt_spdp_wr == 0);
    assign o_cnt_sedp_pub_wr_elapsed = (cnt_sedp_pub_wr == 0);
    assign o_cnt_sedp_sub_wr_elapsed = (cnt_sedp_sub_wr == 0);
    assign o_cnt_sedp_pub_hb_elapsed = (cnt_sedp_pub_hb == 0);
    assign o_cnt_sedp_sub_hb_elapsed = (cnt_sedp_sub_hb == 0);
    assign o_cnt_sedp_pub_an_elapsed = (cnt_sedp_pub_an == 0);
    assign o_cnt_sedp_sub_an_elapsed = (cnt_sedp_sub_an == 0);
    assign o_cnt_app_wr_elapsed      = (cnt_app_wr == 0);

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            cnt_prescaler <= 0;
            cnt_interval <= 0;
            cnt_spdp_wr <= 0;
            cnt_sedp_pub_wr <= 0;
            cnt_sedp_sub_wr <= 0;
            cnt_sedp_pub_hb <= 0;
            cnt_sedp_sub_hb <= 0;
            cnt_sedp_pub_an <= 0;
            cnt_sedp_sub_an <= 0;
            cnt_app_wr <= 0;
        end else begin
            cnt_prescaler <= cnt_prescaler + 1;

            if (i_cnt_interval_set)
                cnt_interval <= TX_INTERVAL_COUNT;
            else if (cnt_prescaler == 0 && cnt_interval != 0)
                cnt_interval <= cnt_interval - 1;

            if (i_cnt_spdp_wr_set)
                cnt_spdp_wr <= TX_PERIOD_SPDP_WR_COUNT;
            else if (cnt_prescaler == 0 && cnt_spdp_wr != 0)
                cnt_spdp_wr <= cnt_spdp_wr - 1;

            if (i_cnt_sedp_pub_wr_set)
                cnt_sedp_pub_wr <= TX_PERIOD_SEDP_PUB_WR_COUNT;
            else if (cnt_prescaler == 0 && cnt_sedp_pub_wr != 0)
                cnt_sedp_pub_wr <= cnt_sedp_pub_wr - 1;

            if (i_cnt_sedp_sub_wr_set)
                cnt_sedp_sub_wr <= TX_PERIOD_SEDP_SUB_WR_COUNT;
            else if (cnt_prescaler == 0 && cnt_sedp_sub_wr != 0)
                cnt_sedp_sub_wr <= cnt_sedp_sub_wr - 1;

            if (i_cnt_sedp_pub_hb_set)
                cnt_sedp_pub_hb <= TX_PERIOD_SEDP_PUB_HB_COUNT;
            else if (cnt_prescaler == 0 && cnt_sedp_pub_hb != 0)
                cnt_sedp_pub_hb <= cnt_sedp_pub_hb - 1;

            if (i_cnt_sedp_sub_hb_set)
                cnt_sedp_sub_hb <= TX_PERIOD_SEDP_SUB_HB_COUNT;
            else if (cnt_prescaler == 0 && cnt_sedp_sub_hb != 0)
                cnt_sedp_sub_hb <= cnt_sedp_sub_hb - 1;

            if (i_cnt_sedp_pub_an_set)
                cnt_sedp_pub_an <= TX_PERIOD_SEDP_PUB_AN_COUNT;
            else if (cnt_prescaler == 0 && cnt_sedp_pub_an != 0)
                cnt_sedp_pub_an <= cnt_sedp_pub_an - 1;

            if (i_cnt_sedp_sub_an_set)
                cnt_sedp_sub_an <= TX_PERIOD_SEDP_SUB_AN_COUNT;
            else if (cnt_prescaler == 0 && cnt_sedp_sub_an != 0)
                cnt_sedp_sub_an <= cnt_sedp_sub_an - 1;

            if (i_cnt_app_wr_set)
                cnt_app_wr <= TX_PERIOD_APP_WR_COUNT;
            else if (cnt_prescaler == 0 && cnt_app_wr != 0)
                cnt_app_wr <= cnt_app_wr - 1;
        end
    end
endmodule

`resetall
