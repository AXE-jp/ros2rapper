// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none

`include "ros2_config.vh"

module ros2rapper #(
    parameter SET_TX_PERIOD_BY_PARAMETER  = 0,
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
    input  wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2pub_en,
    input  wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2sub_en,

    input  wire [7:0] rx_fifo_dout,
    input  wire       rx_fifo_empty,
    output wire       rx_fifo_rd_en,

    output wire [7:0] tx_fifo_din,
    input  wire       tx_fifo_full,
    output wire       tx_fifo_wr_en,

    input  wire [31:0] ip_addr,
    input  wire [31:0] subnet_mask,

    input  wire [15:0] ros2_vendor_id,
    input  wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name,
    input  wire [7:0] ros2_node_name_len,
    input  wire [15:0] ros2_node_udp_port,
    input  wire [15:0] ros2_port_num_seed,

    input  wire [31:0] ros2_fragment_expiration,
    input  wire [95:0] ros2_guid_prefix,
    input  wire [31:0] ros2_participant_lease_duration_seconds,
    input  wire [31:0] ros2_participant_lease_duration_fraction,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name_0,
    input  wire [7:0] ros2_pub_topic_name_len_0,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name_0,
    input  wire [7:0] ros2_pub_topic_type_name_len_0,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name_1,
    input  wire [7:0] ros2_pub_topic_name_len_1,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name_1,
    input  wire [7:0] ros2_pub_topic_type_name_len_1,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name_2,
    input  wire [7:0] ros2_pub_topic_name_len_2,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name_2,
    input  wire [7:0] ros2_pub_topic_type_name_len_2,

    input  wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name_3,
    input  wire [7:0] ros2_pub_topic_name_len_3,
    input  wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name_3,
    input  wire [7:0] ros2_pub_topic_type_name_len_3,

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

`ifdef ROS2_PUB_DATA_FF
    input  wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data_0,
    input  wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data_1,
    input  wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data_2,
    input  wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data_3,
`endif
`ifdef ROS2_PUB_DATA_RAM
    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_0_addr,
    output wire ros2_pub_app_data_0_ce,
    input  wire [31:0] ros2_pub_app_data_0_rdata,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_1_addr,
    output wire ros2_pub_app_data_1_ce,
    input  wire [31:0] ros2_pub_app_data_1_rdata,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_2_addr,
    output wire ros2_pub_app_data_2_ce,
    input  wire [31:0] ros2_pub_app_data_2_rdata,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_3_addr,
    output wire ros2_pub_app_data_3_ce,
    input  wire [31:0] ros2_pub_app_data_3_rdata,
`endif

    input  wire [`ROS2_APP_DATA_LEN_WIDTH-1:0] ros2_pub_app_data_len_0,
    input  wire [`ROS2_APP_DATA_LEN_WIDTH-1:0] ros2_pub_app_data_len_1,
    input  wire [`ROS2_APP_DATA_LEN_WIDTH-1:0] ros2_pub_app_data_len_2,
    input  wire [`ROS2_APP_DATA_LEN_WIDTH-1:0] ros2_pub_app_data_len_3,

    input  wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2_pub_app_data_req,
    input  wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2_pub_app_data_rel,
    output wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2_pub_app_data_ack,
    output wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2_pub_app_data_nack,
    output wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2_pub_app_data_grant,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_0_addr,
    output wire ros2_sub_app_data_0_ce,
    output wire ros2_sub_app_data_0_we,
    output wire [7:0] ros2_sub_app_data_0_wdata,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_1_addr,
    output wire ros2_sub_app_data_1_ce,
    output wire ros2_sub_app_data_1_we,
    output wire [7:0] ros2_sub_app_data_1_wdata,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_2_addr,
    output wire ros2_sub_app_data_2_ce,
    output wire ros2_sub_app_data_2_we,
    output wire [7:0] ros2_sub_app_data_2_wdata,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_3_addr,
    output wire ros2_sub_app_data_3_ce,
    output wire ros2_sub_app_data_3_we,
    output wire [7:0] ros2_sub_app_data_3_wdata,

    output wire [63:0] ros2_sub_app_data_recvinfo_din,
    input  wire ros2_sub_app_data_recvinfo_full_n,
    output wire ros2_sub_app_data_recvinfo_write,

    input  wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2_sub_app_data_req,
    input  wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2_sub_app_data_rel,
    output wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2_sub_app_data_ack,
    output wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2_sub_app_data_nack,
    output wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2_sub_app_data_grant,

    output wire ros2_cnt_interval_set,
    output wire ros2_cnt_spdp_wr_set,
    output wire ros2_cnt_sedp_pub_wr_set,
    output wire ros2_cnt_sedp_sub_wr_set,
    output wire ros2_cnt_sedp_pub_hb_set,
    output wire ros2_cnt_sedp_sub_hb_set,
    output wire ros2_cnt_sedp_pub_an_set,
    output wire ros2_cnt_sedp_sub_an_set,
    output wire ros2_cnt_app_wr_set,

    input wire ros2_cnt_interval_elapsed,
    input wire ros2_cnt_spdp_wr_elapsed,
    input wire ros2_cnt_sedp_pub_wr_elapsed,
    input wire ros2_cnt_sedp_sub_wr_elapsed,
    input wire ros2_cnt_sedp_pub_hb_elapsed,
    input wire ros2_cnt_sedp_sub_hb_elapsed,
    input wire ros2_cnt_sedp_pub_an_elapsed,
    input wire ros2_cnt_sedp_sub_an_elapsed,
    input wire ros2_cnt_app_wr_elapsed,

    input wire [31:0] ros2_timestamp_increment,

    output wire [$clog2(`ROS2_SEDP_READER_MAX+1)-1:0] ros2_sedp_reader_cnt,
    output wire [$clog2(`ROS2_APP_READER_MAX+1)-1:0] ros2_app_reader_cnt,

`ifdef ROS2_SEDP_READER_TBL_RAM
    output wire [$clog2(`ROS2_SEDP_READER_MAX*11)-1:0] sedp_reader_tbl_mem_addr,
    output wire sedp_reader_tbl_mem_ce,
    output wire sedp_reader_tbl_mem_we,
    output wire [63:0] sedp_reader_tbl_mem_wdata,
    input  wire [63:0] sedp_reader_tbl_mem_rdata,
`endif

`ifdef ROS2_APP_READER_TBL_RAM
    output wire [$clog2(`ROS2_APP_READER_MAX)-1:0] app_reader_tbl_mem_addr,
    output wire app_reader_tbl_mem_ce,
    output wire app_reader_tbl_mem_we,
    output wire [63:0] app_reader_tbl_mem_wdata,
    input  wire [63:0] app_reader_tbl_mem_rdata,
`endif

    output wire [`PAYLOADSMEM_AWIDTH-1:0] ip_payloadsmem_addr,
    output wire ip_payloadsmem_ce,
    output wire ip_payloadsmem_we,
    output wire [7:0] ip_payloadsmem_wdata,
    input  wire [7:0] ip_payloadsmem_rdata
);

wire [`ROS2_PUB_TOPICS_MAX-1:0] pub_enable = en ? ros2pub_en : {`ROS2_PUB_TOPICS_MAX{1'b0}};
wire [`ROS2_SUB_TOPICS_MAX-1:0] sub_enable = en ? ros2sub_en : {`ROS2_SUB_TOPICS_MAX{1'b0}};

wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2_pub_app_data_ip_req;
wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2_pub_app_data_ip_rel;
wire [`ROS2_PUB_TOPICS_MAX-1:0] ros2_pub_app_data_ip_grant;

genvar iter;
generate
    for (iter = 0; iter < `ROS2_PUB_TOPICS_MAX; iter = iter+1) begin : PUB_APP_DATA_HS
        app_data_arbiter pub_app_data_arbiter(
            .i_clk(clk), .i_rst_n(rst_n), .i_en(en),
            .i_app_data_ip_req(ros2_pub_app_data_ip_req[iter]),
            .i_app_data_ip_rel(ros2_pub_app_data_ip_rel[iter]),
            .o_app_data_ip_grant(ros2_pub_app_data_ip_grant[iter]),
            .i_app_data_user_req(ros2_pub_app_data_req[iter]),
            .i_app_data_user_rel(ros2_pub_app_data_rel[iter]),
            .o_app_data_user_ack(ros2_pub_app_data_ack[iter]),
            .o_app_data_user_nack(ros2_pub_app_data_nack[iter]),
            .o_app_data_user_grant(ros2_pub_app_data_grant[iter])
        );
    end
endgenerate

wire ros2_sub_app_data_ip_req_valid;
wire ros2_sub_app_data_ip_rel_valid;

wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2_sub_app_data_ip_req;
wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2_sub_app_data_ip_rel;
wire [`ROS2_SUB_TOPICS_MAX-1:0] ros2_sub_app_data_ip_grant;

generate
    for (iter = 0; iter < `ROS2_SUB_TOPICS_MAX; iter = iter+1) begin : SUB_APP_DATA_HS
        app_data_arbiter sub_app_data_arbiter(
            .i_clk(clk), .i_rst_n(rst_n), .i_en(en),
            .i_app_data_ip_req(ros2_sub_app_data_ip_req_valid & ros2_sub_app_data_ip_req[iter]),
            .i_app_data_ip_rel(ros2_sub_app_data_ip_rel_valid & ros2_sub_app_data_ip_rel[iter]),
            .o_app_data_ip_grant(ros2_sub_app_data_ip_grant[iter]),
            .i_app_data_user_req(ros2_sub_app_data_req[iter]),
            .i_app_data_user_rel(ros2_sub_app_data_rel[iter]),
            .o_app_data_user_ack(ros2_sub_app_data_ack[iter]),
            .o_app_data_user_nack(ros2_sub_app_data_nack[iter]),
            .o_app_data_user_grant(ros2_sub_app_data_grant[iter])
        );
    end
endgenerate

// local_timestamp[63:32] is time in second and local_timestamp[31:0] is the fractional part.
reg [63:0] local_timestamp;
// How much local_timestamp increases in each cycle.
wire [31:0] local_timestamp_increment;
generate
    if (SET_TX_PERIOD_BY_PARAMETER) begin
        assign local_timestamp_increment = (34'h200000000 + ROS2CLK_HZ) / (2 * ROS2CLK_HZ);
    end else begin
        assign local_timestamp_increment = ros2_timestamp_increment;
    end
endgenerate
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        local_timestamp <= 64'd0;
    end else begin
        local_timestamp <= local_timestamp + local_timestamp_increment;
    end
end

localparam SEDP_READER_CNT_WIDTH = $clog2(`ROS2_SEDP_READER_MAX+1);
reg  [SEDP_READER_CNT_WIDTH-1:0] r_sedp_reader_cnt;
wire [SEDP_READER_CNT_WIDTH-1:0] w_sedp_reader_cnt;
wire w_sedp_reader_cnt_valid;
assign ros2_sedp_reader_cnt = r_sedp_reader_cnt;

localparam APP_READER_CNT_WIDTH = $clog2(`ROS2_APP_READER_MAX+1);
reg  [APP_READER_CNT_WIDTH-1:0] r_app_reader_cnt;
wire [APP_READER_CNT_WIDTH-1:0] w_app_reader_cnt;
wire w_app_reader_cnt_valid;
assign ros2_app_reader_cnt = r_app_reader_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_sedp_reader_cnt <= {SEDP_READER_CNT_WIDTH{1'b0}};
        r_app_reader_cnt <= {APP_READER_CNT_WIDTH{1'b0}};
    end else begin
        if (w_sedp_reader_cnt_valid)
            r_sedp_reader_cnt <= w_sedp_reader_cnt;
        if (w_app_reader_cnt_valid)
            r_app_reader_cnt <= w_app_reader_cnt;
    end
end

wire [`ROS2_RTPS_DATA_WIDTH-1:0] ros2_rtps_data;
wire ros2_rtps_data_valid;
wire ros2_rtps_data_ready;

wire [`ROS2_MESSAGE_METADATA_WIDTH-1:0] ros2_msg_metadata;
wire ros2_msg_metadata_valid;
wire ros2_msg_metadata_ready;

wire cnt_interval_set;
wire cnt_spdp_wr_set;
wire cnt_sedp_pub_wr_set;
wire cnt_sedp_sub_wr_set;
wire cnt_sedp_pub_hb_set;
wire cnt_sedp_sub_hb_set;
wire cnt_sedp_pub_an_set;
wire cnt_sedp_sub_an_set;
wire cnt_app_wr_set;

wire cnt_interval_elapsed;
wire cnt_spdp_wr_elapsed;
wire cnt_sedp_pub_wr_elapsed;
wire cnt_sedp_sub_wr_elapsed;
wire cnt_sedp_pub_hb_elapsed;
wire cnt_sedp_sub_hb_elapsed;
wire cnt_sedp_pub_an_elapsed;
wire cnt_sedp_sub_an_elapsed;
wire cnt_app_wr_elapsed;

generate
    if (SET_TX_PERIOD_BY_PARAMETER) begin : ROS2_INST_BLOCK_0
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

            .i_cnt_interval_set(cnt_interval_set),
            .i_cnt_spdp_wr_set(cnt_spdp_wr_set),
            .i_cnt_sedp_pub_wr_set(cnt_sedp_pub_wr_set),
            .i_cnt_sedp_sub_wr_set(cnt_sedp_sub_wr_set),
            .i_cnt_sedp_pub_hb_set(cnt_sedp_pub_hb_set),
            .i_cnt_sedp_sub_hb_set(cnt_sedp_sub_hb_set),
            .i_cnt_sedp_pub_an_set(cnt_sedp_pub_an_set),
            .i_cnt_sedp_sub_an_set(cnt_sedp_sub_an_set),
            .i_cnt_app_wr_set(cnt_app_wr_set),

            .o_cnt_interval_elapsed(cnt_interval_elapsed),
            .o_cnt_spdp_wr_elapsed(cnt_spdp_wr_elapsed),
            .o_cnt_sedp_pub_wr_elapsed(cnt_sedp_pub_wr_elapsed),
            .o_cnt_sedp_sub_wr_elapsed(cnt_sedp_sub_wr_elapsed),
            .o_cnt_sedp_pub_hb_elapsed(cnt_sedp_pub_hb_elapsed),
            .o_cnt_sedp_sub_hb_elapsed(cnt_sedp_sub_hb_elapsed),
            .o_cnt_sedp_pub_an_elapsed(cnt_sedp_pub_an_elapsed),
            .o_cnt_sedp_sub_an_elapsed(cnt_sedp_sub_an_elapsed),
            .o_cnt_app_wr_elapsed(cnt_app_wr_elapsed)
        );
    end else begin : ROS2_INST_BLOCK_1
        assign ros2_cnt_interval_set = cnt_interval_set;
        assign ros2_cnt_spdp_wr_set = cnt_spdp_wr_set;
        assign ros2_cnt_sedp_pub_wr_set = cnt_sedp_pub_wr_set;
        assign ros2_cnt_sedp_sub_wr_set = cnt_sedp_sub_wr_set;
        assign ros2_cnt_sedp_pub_hb_set = cnt_sedp_pub_hb_set;
        assign ros2_cnt_sedp_sub_hb_set = cnt_sedp_sub_hb_set;
        assign ros2_cnt_sedp_pub_an_set = cnt_sedp_pub_an_set;
        assign ros2_cnt_sedp_sub_an_set = cnt_sedp_sub_an_set;
        assign ros2_cnt_app_wr_set = cnt_app_wr_set;

        assign cnt_interval_elapsed = ros2_cnt_interval_elapsed;
        assign cnt_spdp_wr_elapsed = ros2_cnt_spdp_wr_elapsed;
        assign cnt_sedp_pub_wr_elapsed = ros2_cnt_sedp_pub_wr_elapsed;
        assign cnt_sedp_sub_wr_elapsed = ros2_cnt_sedp_sub_wr_elapsed;
        assign cnt_sedp_pub_hb_elapsed = ros2_cnt_sedp_pub_hb_elapsed;
        assign cnt_sedp_sub_hb_elapsed = ros2_cnt_sedp_sub_hb_elapsed;
        assign cnt_sedp_pub_an_elapsed = ros2_cnt_sedp_pub_an_elapsed;
        assign cnt_sedp_sub_an_elapsed = ros2_cnt_sedp_sub_an_elapsed;
        assign cnt_app_wr_elapsed = ros2_cnt_app_wr_elapsed;
    end
endgenerate

`ifdef ROS2RAPPER_HLS_VITIS

wire [15:0] pre_ip_tdata;
wire pre_ip_tvalid;
wire pre_ip_tready;

pre_ip_in
pre_ip_in (
    .ap_clk(clk),
    .ap_rst_n(rst_n),
    .in_r_TDATA(rx_fifo_dout),
    .in_r_TVALID(~rx_fifo_empty),
    .in_r_TREADY(rx_fifo_rd_en),
    .out_r_TDATA(pre_ip_tdata),
    .out_r_TVALID(pre_ip_tvalid),
    .out_r_TREADY(pre_ip_tready)
);

wire [15:0] udp_ip_tdata;
wire udp_ip_tvalid;
wire udp_ip_tready;

udp_ip_in
udp_ip_in (
    .ap_clk(clk),
    .ap_rst_n(rst_n),
    .in_r_TDATA(pre_ip_tdata),
    .in_r_TVALID(pre_ip_tvalid),
    .in_r_TREADY(pre_ip_tready),
    .out_r_TDATA(udp_ip_tdata),
    .out_r_TVALID(udp_ip_tvalid),
    .out_r_TREADY(udp_ip_tready),
    .ip_payloads_address0(ip_payloadsmem_addr),
    .ip_payloads_ce0(ip_payloadsmem_ce),
    .ip_payloads_we0(ip_payloadsmem_we),
    .ip_payloads_d0(ip_payloadsmem_wdata),
    .ip_payloads_q0(ip_payloadsmem_rdata),
    .fragment_expiration(ros2_fragment_expiration),
    .error(),
    .error_ap_vld()
);

ros2_receiver
ros2_receiver (
    .ap_clk(clk),
    .ap_rst_n(rst_n),

    .in_r_TDATA(udp_ip_tdata),
    .in_r_TVALID(udp_ip_tvalid),
    .in_r_TREADY(udp_ip_tready),

    .out_r_TDATA(ros2_rtps_data),
    .out_r_TREADY(ros2_rtps_data_ready),
    .out_r_TVALID(ros2_rtps_data_valid),

    .pub_enable(pub_enable),
    .sub_enable(sub_enable),

    .sub_app_data_req_ap_vld(ros2_sub_app_data_ip_req_valid),
    .sub_app_data_req(ros2_sub_app_data_ip_req),
    .sub_app_data_rel_ap_vld(ros2_sub_app_data_ip_rel_valid),
    .sub_app_data_rel(ros2_sub_app_data_ip_rel),
    .sub_app_data_grant(ros2_sub_app_data_ip_grant),

    .sub_app_data_0_address0(ros2_sub_app_data_0_addr),
    .sub_app_data_0_ce0(ros2_sub_app_data_0_ce),
    .sub_app_data_0_we0(ros2_sub_app_data_0_we),
    .sub_app_data_0_d0(ros2_sub_app_data_0_wdata),

    .sub_app_data_1_address0(ros2_sub_app_data_1_addr),
    .sub_app_data_1_ce0(ros2_sub_app_data_1_ce),
    .sub_app_data_1_we0(ros2_sub_app_data_1_we),
    .sub_app_data_1_d0(ros2_sub_app_data_1_wdata),

    .sub_app_data_2_address0(ros2_sub_app_data_2_addr),
    .sub_app_data_2_ce0(ros2_sub_app_data_2_ce),
    .sub_app_data_2_we0(ros2_sub_app_data_2_we),
    .sub_app_data_2_d0(ros2_sub_app_data_2_wdata),

    .sub_app_data_3_address0(ros2_sub_app_data_3_addr),
    .sub_app_data_3_ce0(ros2_sub_app_data_3_ce),
    .sub_app_data_3_we0(ros2_sub_app_data_3_we),
    .sub_app_data_3_d0(ros2_sub_app_data_3_wdata),

    .sub_app_data_recvinfo_TDATA(ros2_sub_app_data_recvinfo_din),
    .sub_app_data_recvinfo_TVALID(ros2_sub_app_data_recvinfo_write),
    .sub_app_data_recvinfo_TREADY(ros2_sub_app_data_recvinfo_full_n),

    .conf_ip_addr(ip_addr),
    .conf_subnet_mask(subnet_mask),
    .conf_port_num_seed(ros2_port_num_seed),
    .conf_guid_prefix(ros2_guid_prefix),

    .conf_pub_topic_name_0(ros2_pub_topic_name_0),
    .conf_pub_topic_name_1(ros2_pub_topic_name_1),
    .conf_pub_topic_name_2(ros2_pub_topic_name_2),
    .conf_pub_topic_name_3(ros2_pub_topic_name_3),
    .conf_pub_topic_name_len_0(ros2_pub_topic_name_len_0),
    .conf_pub_topic_name_len_1(ros2_pub_topic_name_len_1),
    .conf_pub_topic_name_len_2(ros2_pub_topic_name_len_2),
    .conf_pub_topic_name_len_3(ros2_pub_topic_name_len_3),

    .conf_pub_topic_type_name_0(ros2_pub_topic_type_name_0),
    .conf_pub_topic_type_name_1(ros2_pub_topic_type_name_1),
    .conf_pub_topic_type_name_2(ros2_pub_topic_type_name_2),
    .conf_pub_topic_type_name_3(ros2_pub_topic_type_name_3),
    .conf_pub_topic_type_name_len_0(ros2_pub_topic_type_name_len_0),
    .conf_pub_topic_type_name_len_1(ros2_pub_topic_type_name_len_1),
    .conf_pub_topic_type_name_len_2(ros2_pub_topic_type_name_len_2),
    .conf_pub_topic_type_name_len_3(ros2_pub_topic_type_name_len_3),

    .conf_sub_topic_name_0(ros2_sub_topic_name_0),
    .conf_sub_topic_name_1(ros2_sub_topic_name_1),
    .conf_sub_topic_name_2(ros2_sub_topic_name_2),
    .conf_sub_topic_name_3(ros2_sub_topic_name_3),
    .conf_sub_topic_name_len_0(ros2_sub_topic_name_len_0),
    .conf_sub_topic_name_len_1(ros2_sub_topic_name_len_1),
    .conf_sub_topic_name_len_2(ros2_sub_topic_name_len_2),
    .conf_sub_topic_name_len_3(ros2_sub_topic_name_len_3),

    .conf_sub_topic_type_name_0(ros2_sub_topic_type_name_0),
    .conf_sub_topic_type_name_1(ros2_sub_topic_type_name_1),
    .conf_sub_topic_type_name_2(ros2_sub_topic_type_name_2),
    .conf_sub_topic_type_name_3(ros2_sub_topic_type_name_3),
    .conf_sub_topic_type_name_len_0(ros2_sub_topic_type_name_len_0),
    .conf_sub_topic_type_name_len_1(ros2_sub_topic_type_name_len_1),
    .conf_sub_topic_type_name_len_2(ros2_sub_topic_type_name_len_2),
    .conf_sub_topic_type_name_len_3(ros2_sub_topic_type_name_len_3)
);

ros2_main
ros2_main (
    .ap_clk(clk),
    .ap_rst_n(rst_n),

`ifdef ROS2_SEDP_READER_TBL_RAM
    .sedp_reader_tbl_address0(sedp_reader_tbl_mem_addr),
    .sedp_reader_tbl_ce0(sedp_reader_tbl_mem_ce),
    .sedp_reader_tbl_we0(sedp_reader_tbl_mem_we),
    .sedp_reader_tbl_d0(sedp_reader_tbl_mem_wdata),
    .sedp_reader_tbl_q0(sedp_reader_tbl_mem_rdata),
`endif

`ifdef ROS2_APP_READER_TBL_RAM
    .app_reader_tbl_address0(app_reader_tbl_mem_addr),
    .app_reader_tbl_ce0(app_reader_tbl_mem_ce),
    .app_reader_tbl_we0(app_reader_tbl_mem_we),
    .app_reader_tbl_d0(app_reader_tbl_mem_wdata),
    .app_reader_tbl_q0(app_reader_tbl_mem_rdata),
`endif

    .pub_enable(pub_enable),
    .sub_enable(sub_enable),

    .in_r_TDATA(ros2_rtps_data),
    .in_r_TREADY(ros2_rtps_data_ready),
    .in_r_TVALID(ros2_rtps_data_valid),

    .out_r_TDATA(ros2_msg_metadata),
    .out_r_TREADY(ros2_msg_metadata_ready),
    .out_r_TVALID(ros2_msg_metadata_valid),

    .conf_port_num_seed(ros2_port_num_seed),
    .conf_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
    .conf_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),

    .cnt_interval_set(),
    .cnt_interval_set_ap_vld(cnt_interval_set),
    .cnt_spdp_wr_set(),
    .cnt_spdp_wr_set_ap_vld(cnt_spdp_wr_set),
    .cnt_sedp_pub_wr_set(),
    .cnt_sedp_pub_wr_set_ap_vld(cnt_sedp_pub_wr_set),
    .cnt_sedp_sub_wr_set(),
    .cnt_sedp_sub_wr_set_ap_vld(cnt_sedp_sub_wr_set),
    .cnt_sedp_pub_hb_set(),
    .cnt_sedp_pub_hb_set_ap_vld(cnt_sedp_pub_hb_set),
    .cnt_sedp_sub_hb_set(),
    .cnt_sedp_sub_hb_set_ap_vld(cnt_sedp_sub_hb_set),
    .cnt_sedp_pub_an_set(),
    .cnt_sedp_pub_an_set_ap_vld(cnt_sedp_pub_an_set),
    .cnt_sedp_sub_an_set(),
    .cnt_sedp_sub_an_set_ap_vld(cnt_sedp_sub_an_set),
    .cnt_app_wr_set(),
    .cnt_app_wr_set_ap_vld(cnt_app_wr_set),

    .cnt_interval_elapsed(cnt_interval_elapsed),
    .cnt_interval_elapsed_ap_ack(),
    .cnt_spdp_wr_elapsed(cnt_spdp_wr_elapsed),
    .cnt_spdp_wr_elapsed_ap_ack(),
    .cnt_sedp_pub_wr_elapsed(cnt_sedp_pub_wr_elapsed),
    .cnt_sedp_pub_wr_elapsed_ap_ack(),
    .cnt_sedp_sub_wr_elapsed(cnt_sedp_sub_wr_elapsed),
    .cnt_sedp_sub_wr_elapsed_ap_ack(),
    .cnt_sedp_pub_hb_elapsed(cnt_sedp_pub_hb_elapsed),
    .cnt_sedp_pub_hb_elapsed_ap_ack(),
    .cnt_sedp_sub_hb_elapsed(cnt_sedp_sub_hb_elapsed),
    .cnt_sedp_sub_hb_elapsed_ap_ack(),
    .cnt_sedp_pub_an_elapsed(cnt_sedp_pub_an_elapsed),
    .cnt_sedp_pub_an_elapsed_ap_ack(),
    .cnt_sedp_sub_an_elapsed(cnt_sedp_sub_an_elapsed),
    .cnt_sedp_sub_an_elapsed_ap_ack(),
    .cnt_app_wr_elapsed(cnt_app_wr_elapsed),
    .cnt_app_wr_elapsed_ap_ack(),

    .timestamp_i64(local_timestamp),
    .sedp_reader_cnt(w_sedp_reader_cnt),
    .sedp_reader_cnt_ap_vld(w_sedp_reader_cnt_valid),
    .app_reader_cnt(w_app_reader_cnt),
    .app_reader_cnt_ap_vld(w_app_reader_cnt_valid)
);

ros2_sender
ros2_sender (
    .ap_clk(clk),
    .ap_rst_n(rst_n),

    .in_r_TDATA(ros2_msg_metadata),
    .in_r_TVALID(ros2_msg_metadata_valid),
    .in_r_TREADY(ros2_msg_metadata_ready),

    .out_r_din(tx_fifo_din),
    .out_r_full_n(~tx_fifo_full),
    .out_r_write(tx_fifo_wr_en),

    .conf_ip_addr(ip_addr),
    .conf_vendor_id(ros2_vendor_id),
    .conf_node_name(ros2_node_name),
    .conf_node_name_len(ros2_node_name_len),
    .conf_node_udp_port({ros2_node_udp_port[7:0], ros2_node_udp_port[15:8]}),
    .conf_guid_prefix(ros2_guid_prefix),

    .conf_pub_topic_name_0(ros2_pub_topic_name_0),
    .conf_pub_topic_name_len_0(ros2_pub_topic_name_len_0),
    .conf_pub_topic_type_name_0(ros2_pub_topic_type_name_0),
    .conf_pub_topic_type_name_len_0(ros2_pub_topic_type_name_len_0),
    .conf_pub_topic_name_1(ros2_pub_topic_name_1),
    .conf_pub_topic_name_len_1(ros2_pub_topic_name_len_1),
    .conf_pub_topic_type_name_1(ros2_pub_topic_type_name_1),
    .conf_pub_topic_type_name_len_1(ros2_pub_topic_type_name_len_1),
    .conf_pub_topic_name_2(ros2_pub_topic_name_2),
    .conf_pub_topic_name_len_2(ros2_pub_topic_name_len_2),
    .conf_pub_topic_type_name_2(ros2_pub_topic_type_name_2),
    .conf_pub_topic_type_name_len_2(ros2_pub_topic_type_name_len_2),
    .conf_pub_topic_name_3(ros2_pub_topic_name_3),
    .conf_pub_topic_name_len_3(ros2_pub_topic_name_len_3),
    .conf_pub_topic_type_name_3(ros2_pub_topic_type_name_3),
    .conf_pub_topic_type_name_len_3(ros2_pub_topic_type_name_len_3),
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

`ifdef ROS2_PUB_DATA_FF
    .pub_app_data_0_dout(ros2_pub_app_data_0),
    .pub_app_data_0_empty_n(1'b1),
    .pub_app_data_0_read(),

    .pub_app_data_1_dout(ros2_pub_app_data_1),
    .pub_app_data_1_empty_n(1'b1),
    .pub_app_data_1_read(),

    .pub_app_data_2_dout(ros2_pub_app_data_2),
    .pub_app_data_2_empty_n(1'b1),
    .pub_app_data_2_read(),

    .pub_app_data_3_dout(ros2_pub_app_data_3),
    .pub_app_data_3_empty_n(1'b1),
    .pub_app_data_3_read(),
`endif
`ifdef ROS2_PUB_DATA_RAM
    .pub_app_data_0_address0(ros2_pub_app_data_0_addr),
    .pub_app_data_0_ce0(ros2_pub_app_data_0_ce),
    .pub_app_data_0_q0(ros2_pub_app_data_0_rdata),

    .pub_app_data_1_address0(ros2_pub_app_data_1_addr),
    .pub_app_data_1_ce0(ros2_pub_app_data_1_ce),
    .pub_app_data_1_q0(ros2_pub_app_data_1_rdata),

    .pub_app_data_2_address0(ros2_pub_app_data_2_addr),
    .pub_app_data_2_ce0(ros2_pub_app_data_2_ce),
    .pub_app_data_2_q0(ros2_pub_app_data_2_rdata),

    .pub_app_data_3_address0(ros2_pub_app_data_3_addr),
    .pub_app_data_3_ce0(ros2_pub_app_data_3_ce),
    .pub_app_data_3_q0(ros2_pub_app_data_3_rdata),
`endif

    .pub_app_data_len_0_dout(ros2_pub_app_data_len_0),
    .pub_app_data_len_0_empty_n(1'b1),
    .pub_app_data_len_0_read(),
    .pub_app_data_req_0_ap_vld(ros2_pub_app_data_ip_req[0]),
    .pub_app_data_req_0(),
    .pub_app_data_rel_0_ap_vld(ros2_pub_app_data_ip_rel[0]),
    .pub_app_data_rel_0(),
    .pub_app_data_grant_0({7'd0, ros2_pub_app_data_ip_grant[0]}),
    .pub_app_data_grant_0_ap_ack(),

    .pub_app_data_len_1_dout(ros2_pub_app_data_len_1),
    .pub_app_data_len_1_empty_n(1'b1),
    .pub_app_data_len_1_read(),
    .pub_app_data_req_1_ap_vld(ros2_pub_app_data_ip_req[1]),
    .pub_app_data_req_1(),
    .pub_app_data_rel_1_ap_vld(ros2_pub_app_data_ip_rel[1]),
    .pub_app_data_rel_1(),
    .pub_app_data_grant_1({7'd0, ros2_pub_app_data_ip_grant[1]}),
    .pub_app_data_grant_1_ap_ack(),

    .pub_app_data_len_2_dout(ros2_pub_app_data_len_2),
    .pub_app_data_len_2_empty_n(1'b1),
    .pub_app_data_len_2_read(),
    .pub_app_data_req_2_ap_vld(ros2_pub_app_data_ip_req[2]),
    .pub_app_data_req_2(),
    .pub_app_data_rel_2_ap_vld(ros2_pub_app_data_ip_rel[2]),
    .pub_app_data_rel_2(),
    .pub_app_data_grant_2({7'd0, ros2_pub_app_data_ip_grant[2]}),
    .pub_app_data_grant_2_ap_ack(),

    .pub_app_data_len_3_dout(ros2_pub_app_data_len_3),
    .pub_app_data_len_3_empty_n(1'b1),
    .pub_app_data_len_3_read(),
    .pub_app_data_req_3_ap_vld(ros2_pub_app_data_ip_req[3]),
    .pub_app_data_req_3(),
    .pub_app_data_rel_3_ap_vld(ros2_pub_app_data_ip_rel[3]),
    .pub_app_data_rel_3(),
    .pub_app_data_grant_3({7'd0, ros2_pub_app_data_ip_grant[3]}),
    .pub_app_data_grant_3_ap_ack()
);
`elsif ROS2RAPPER_HLS_CWB

wire [8:0] pre_ip_in_tdata;
wire pre_ip_in_tready;
wire pre_ip_in_tvalid;

pre_ip_in
pre_ip_in (
  .clk(clk),
  .rst_n(rst_n),
  .in_TDATA(rx_fifo_dout),
  .in_TREADY(rx_fifo_rd_en),
  .in_TVALID(~rx_fifo_empty),
  .out_TDATA(pre_ip_in_tdata),
  .out_TREADY(pre_ip_in_tready),
  .out_TVALID(pre_ip_in_tvalid)
);

wire [8:0] udp_ip_in_tdata;
wire udp_ip_in_tready;
wire udp_ip_in_tvalid;

udp_ip_in
udp_ip_in (
  .clk(clk),
  .rst_n(rst_n),
  .in_TDATA(pre_ip_in_tdata),
  .in_TREADY(pre_ip_in_tready),
  .in_TVALID(pre_ip_in_tvalid),
  .out_TDATA(udp_ip_in_tdata),
  .out_TREADY(udp_ip_in_tready),
  .out_TVALID(udp_ip_in_tvalid),
  .ip_payloads_CS1(ip_payloadsmem_ce),
  .ip_payloads_AD1(ip_payloadsmem_addr),
  .ip_payloads_WE1(ip_payloadsmem_we),
  .ip_payloads_WD1(ip_payloadsmem_wdata),
  .ip_payloads_RD1(ip_payloadsmem_rdata),
  .fragment_expiration(ros2_fragment_expiration),
  .error_wd(),
  .error_we()
);

ros2_receiver
ros2_receiver (
  .clk(clk),
  .rst_n(rst_n),

  .pub_enable(pub_enable),
  .sub_enable(sub_enable),

  .in_TDATA(udp_ip_in_tdata),
  .in_TREADY(udp_ip_in_tready),
  .in_TVALID(udp_ip_in_tvalid),

  .out_TDATA(ros2_rtps_data),
  .out_TREADY(ros2_rtps_data_ready),
  .out_TVALID(ros2_rtps_data_valid),

  .conf_ip_addr_0_ds0_rd(ip_addr),
  .conf_subnet_mask_0_ds0_rd(subnet_mask),
  .conf_port_num_seed(ros2_port_num_seed),
  .conf_guid_prefix_00_ds00_rd(ros2_guid_prefix),

  .conf_pub_topic_name_0_00_ds0_00_rd(ros2_pub_topic_name_0),
  .conf_pub_topic_name_len_0(ros2_pub_topic_name_len_0),
  .conf_pub_topic_type_name_0_00_ds0_00_rd(ros2_pub_topic_type_name_0),
  .conf_pub_topic_type_name_len_0(ros2_pub_topic_type_name_len_0),

  .conf_pub_topic_name_1_00_ds0_00_rd(ros2_pub_topic_name_1),
  .conf_pub_topic_name_len_1(ros2_pub_topic_name_len_1),
  .conf_pub_topic_type_name_1_00_ds0_00_rd(ros2_pub_topic_type_name_1),
  .conf_pub_topic_type_name_len_1(ros2_pub_topic_type_name_len_1),

  .conf_pub_topic_name_2_00_ds0_00_rd(ros2_pub_topic_name_2),
  .conf_pub_topic_name_len_2(ros2_pub_topic_name_len_2),
  .conf_pub_topic_type_name_2_00_ds0_00_rd(ros2_pub_topic_type_name_2),
  .conf_pub_topic_type_name_len_2(ros2_pub_topic_type_name_len_2),

  .conf_pub_topic_name_3_00_ds0_00_rd(ros2_pub_topic_name_3),
  .conf_pub_topic_name_len_3(ros2_pub_topic_name_len_3),
  .conf_pub_topic_type_name_3_00_ds0_00_rd(ros2_pub_topic_type_name_3),
  .conf_pub_topic_type_name_len_3(ros2_pub_topic_type_name_len_3),

  .conf_sub_topic_name_0_00_ds0_00_rd(ros2_sub_topic_name_0),
  .conf_sub_topic_name_len_0(ros2_sub_topic_name_len_0),
  .conf_sub_topic_type_name_0_00_ds0_00_rd(ros2_sub_topic_type_name_0),
  .conf_sub_topic_type_name_len_0(ros2_sub_topic_type_name_len_0),

  .conf_sub_topic_name_1_00_ds0_00_rd(ros2_sub_topic_name_1),
  .conf_sub_topic_name_len_1(ros2_sub_topic_name_len_1),
  .conf_sub_topic_type_name_1_00_ds0_00_rd(ros2_sub_topic_type_name_1),
  .conf_sub_topic_type_name_len_1(ros2_sub_topic_type_name_len_1),

  .conf_sub_topic_name_2_00_ds0_00_rd(ros2_sub_topic_name_2),
  .conf_sub_topic_name_len_2(ros2_sub_topic_name_len_2),
  .conf_sub_topic_type_name_2_00_ds0_00_rd(ros2_sub_topic_type_name_2),
  .conf_sub_topic_type_name_len_2(ros2_sub_topic_type_name_len_2),

  .conf_sub_topic_name_3_00_ds0_00_rd(ros2_sub_topic_name_3),
  .conf_sub_topic_name_len_3(ros2_sub_topic_name_len_3),
  .conf_sub_topic_type_name_3_00_ds0_00_rd(ros2_sub_topic_type_name_3),
  .conf_sub_topic_type_name_len_3(ros2_sub_topic_type_name_len_3),

  .sub_app_data_0_CS1(ros2_sub_app_data_0_ce),
  .sub_app_data_0_AD1(ros2_sub_app_data_0_addr),
  .sub_app_data_0_WE1(ros2_sub_app_data_0_we),
  .sub_app_data_0_WD1(ros2_sub_app_data_0_wdata),

  .sub_app_data_1_CS1(ros2_sub_app_data_1_ce),
  .sub_app_data_1_AD1(ros2_sub_app_data_1_addr),
  .sub_app_data_1_WE1(ros2_sub_app_data_1_we),
  .sub_app_data_1_WD1(ros2_sub_app_data_1_wdata),

  .sub_app_data_2_CS1(ros2_sub_app_data_2_ce),
  .sub_app_data_2_AD1(ros2_sub_app_data_2_addr),
  .sub_app_data_2_WE1(ros2_sub_app_data_2_we),
  .sub_app_data_2_WD1(ros2_sub_app_data_2_wdata),

  .sub_app_data_3_CS1(ros2_sub_app_data_3_ce),
  .sub_app_data_3_AD1(ros2_sub_app_data_3_addr),
  .sub_app_data_3_WE1(ros2_sub_app_data_3_we),
  .sub_app_data_3_WD1(ros2_sub_app_data_3_wdata),

  .sub_app_data_recvinfo_TDATA(ros2_sub_app_data_recvinfo_din),
  .sub_app_data_recvinfo_TREADY(ros2_sub_app_data_recvinfo_full_n),
  .sub_app_data_recvinfo_TVALID(ros2_sub_app_data_recvinfo_write),

  .sub_app_data_req_we(ros2_sub_app_data_ip_req_valid),
  .sub_app_data_req_wd(ros2_sub_app_data_ip_req),
  .sub_app_data_rel_we(ros2_sub_app_data_ip_rel_valid),
  .sub_app_data_rel_wd(ros2_sub_app_data_ip_rel),
  .sub_app_data_grant(ros2_sub_app_data_ip_grant)
);

ros2_main
ros2_main (
  .clk(clk),
  .rst_n(rst_n),

`ifdef ROS2_SEDP_READER_TBL_RAM
  .sedp_reader_tbl_ram_AD1(sedp_reader_tbl_mem_addr),
  .sedp_reader_tbl_ram_CS1(sedp_reader_tbl_mem_ce),
  .sedp_reader_tbl_ram_WE1(sedp_reader_tbl_mem_we),
  .sedp_reader_tbl_ram_WD1(sedp_reader_tbl_mem_wdata),
  .sedp_reader_tbl_ram_RD1(sedp_reader_tbl_mem_rdata),
`endif

`ifdef ROS2_APP_READER_TBL_RAM
  .app_reader_tbl_ram_AD1(app_reader_tbl_mem_addr),
  .app_reader_tbl_ram_CS1(app_reader_tbl_mem_ce),
  .app_reader_tbl_ram_WE1(app_reader_tbl_mem_we),
  .app_reader_tbl_ram_WD1(app_reader_tbl_mem_wdata),
  .app_reader_tbl_ram_RD1(app_reader_tbl_mem_rdata),
`endif

  .pub_enable(pub_enable),
  .sub_enable(sub_enable),

  .in_TDATA(ros2_rtps_data),
  .in_TREADY(ros2_rtps_data_ready),
  .in_TVALID(ros2_rtps_data_valid),

  .out_TDATA(ros2_msg_metadata),
  .out_TREADY(ros2_msg_metadata_ready),
  .out_TVALID(ros2_msg_metadata_valid),

  .conf_port_num_seed(ros2_port_num_seed),
  .conf_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
  .conf_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),

  .cnt_interval_set_wd(),
  .cnt_interval_set_we(cnt_interval_set),
  .cnt_spdp_wr_set_wd(),
  .cnt_spdp_wr_set_we(cnt_spdp_wr_set),
  .cnt_sedp_pub_wr_set_wd(),
  .cnt_sedp_pub_wr_set_we(cnt_sedp_pub_wr_set),
  .cnt_sedp_sub_wr_set_wd(),
  .cnt_sedp_sub_wr_set_we(cnt_sedp_sub_wr_set),
  .cnt_sedp_pub_hb_set_wd(),
  .cnt_sedp_pub_hb_set_we(cnt_sedp_pub_hb_set),
  .cnt_sedp_sub_hb_set_wd(),
  .cnt_sedp_sub_hb_set_we(cnt_sedp_sub_hb_set),
  .cnt_sedp_pub_an_set_wd(),
  .cnt_sedp_pub_an_set_we(cnt_sedp_pub_an_set),
  .cnt_sedp_sub_an_set_wd(),
  .cnt_sedp_sub_an_set_we(cnt_sedp_sub_an_set),
  .cnt_app_wr_set_wd(),
  .cnt_app_wr_set_we(cnt_app_wr_set),

  .cnt_interval_elapsed(cnt_interval_elapsed),
  .cnt_spdp_wr_elapsed(cnt_spdp_wr_elapsed),
  .cnt_sedp_pub_wr_elapsed(cnt_sedp_pub_wr_elapsed),
  .cnt_sedp_sub_wr_elapsed(cnt_sedp_sub_wr_elapsed),
  .cnt_sedp_pub_hb_elapsed(cnt_sedp_pub_hb_elapsed),
  .cnt_sedp_sub_hb_elapsed(cnt_sedp_sub_hb_elapsed),
  .cnt_sedp_pub_an_elapsed(cnt_sedp_pub_an_elapsed),
  .cnt_sedp_sub_an_elapsed(cnt_sedp_sub_an_elapsed),
  .cnt_app_wr_elapsed(cnt_app_wr_elapsed),

  .timestamp_i64(local_timestamp),
  .sedp_reader_cnt_wd(w_sedp_reader_cnt),
  .sedp_reader_cnt_we(w_sedp_reader_cnt_valid),
  .app_reader_cnt_wd(w_app_reader_cnt),
  .app_reader_cnt_we(w_app_reader_cnt_valid)
);

ros2_sender
ros2_sender (
  .clk(clk),
  .rst_n(rst_n),

  .in_TDATA(ros2_msg_metadata),
  .in_TVALID(ros2_msg_metadata_valid),
  .in_TREADY(ros2_msg_metadata_ready),

  .out_din(tx_fifo_din),
  .out_full(tx_fifo_full),
  .out_wreq(tx_fifo_wr_en),

  .conf_ip_addr_0_ds0_rd(ip_addr),
  .conf_vendor_id_0_ds0_rd(ros2_vendor_id),
  .conf_node_name_00_ds00_rd(ros2_node_name),
  .conf_node_name_len(ros2_node_name_len),
  .conf_node_udp_port_0_ds0_rd(ros2_node_udp_port),
  .conf_guid_prefix_00_ds00_rd(ros2_guid_prefix),

  .conf_pub_topic_name_0_00_ds0_00_rd(ros2_pub_topic_name_0),
  .conf_pub_topic_name_len_0(ros2_pub_topic_name_len_0),
  .conf_pub_topic_type_name_0_00_ds0_00_rd(ros2_pub_topic_type_name_0),
  .conf_pub_topic_type_name_len_0(ros2_pub_topic_type_name_len_0),

  .conf_pub_topic_name_1_00_ds0_00_rd(ros2_pub_topic_name_1),
  .conf_pub_topic_name_len_1(ros2_pub_topic_name_len_1),
  .conf_pub_topic_type_name_1_00_ds0_00_rd(ros2_pub_topic_type_name_1),
  .conf_pub_topic_type_name_len_1(ros2_pub_topic_type_name_len_1),

  .conf_pub_topic_name_2_00_ds0_00_rd(ros2_pub_topic_name_2),
  .conf_pub_topic_name_len_2(ros2_pub_topic_name_len_2),
  .conf_pub_topic_type_name_2_00_ds0_00_rd(ros2_pub_topic_type_name_2),
  .conf_pub_topic_type_name_len_2(ros2_pub_topic_type_name_len_2),

  .conf_pub_topic_name_3_00_ds0_00_rd(ros2_pub_topic_name_3),
  .conf_pub_topic_name_len_3(ros2_pub_topic_name_len_3),
  .conf_pub_topic_type_name_3_00_ds0_00_rd(ros2_pub_topic_type_name_3),
  .conf_pub_topic_type_name_len_3(ros2_pub_topic_type_name_len_3),

  .conf_sub_topic_name_0_00_ds0_00_rd(ros2_sub_topic_name_0),
  .conf_sub_topic_name_len_0(ros2_sub_topic_name_len_0),
  .conf_sub_topic_type_name_0_00_ds0_00_rd(ros2_sub_topic_type_name_0),
  .conf_sub_topic_type_name_len_0(ros2_sub_topic_type_name_len_0),

  .conf_sub_topic_name_1_00_ds0_00_rd(ros2_sub_topic_name_1),
  .conf_sub_topic_name_len_1(ros2_sub_topic_name_len_1),
  .conf_sub_topic_type_name_1_00_ds0_00_rd(ros2_sub_topic_type_name_1),
  .conf_sub_topic_type_name_len_1(ros2_sub_topic_type_name_len_1),

  .conf_sub_topic_name_2_00_ds0_00_rd(ros2_sub_topic_name_2),
  .conf_sub_topic_name_len_2(ros2_sub_topic_name_len_2),
  .conf_sub_topic_type_name_2_00_ds0_00_rd(ros2_sub_topic_type_name_2),
  .conf_sub_topic_type_name_len_2(ros2_sub_topic_type_name_len_2),

  .conf_sub_topic_name_3_00_ds0_00_rd(ros2_sub_topic_name_3),
  .conf_sub_topic_name_len_3(ros2_sub_topic_name_len_3),
  .conf_sub_topic_type_name_3_00_ds0_00_rd(ros2_sub_topic_type_name_3),
  .conf_sub_topic_type_name_len_3(ros2_sub_topic_type_name_len_3),

`ifdef ROS2_PUB_DATA_FF
  .pub_app_data_0_00_ds00_rd(ros2_pub_app_data_0),
  .pub_app_data_1_00_ds00_rd(ros2_pub_app_data_1),
  .pub_app_data_2_00_ds00_rd(ros2_pub_app_data_2),
  .pub_app_data_3_00_ds00_rd(ros2_pub_app_data_3),
`endif
`ifdef ROS2_PUB_DATA_RAM
  .pub_app_data_0_CS1(ros2_pub_app_data_0_ce),
  .pub_app_data_0_AD1(ros2_pub_app_data_0_addr),
  .pub_app_data_0_RD1(ros2_pub_app_data_0_rdata),

  .pub_app_data_1_CS1(ros2_pub_app_data_1_ce),
  .pub_app_data_1_AD1(ros2_pub_app_data_1_addr),
  .pub_app_data_1_RD1(ros2_pub_app_data_1_rdata),

  .pub_app_data_2_CS1(ros2_pub_app_data_2_ce),
  .pub_app_data_2_AD1(ros2_pub_app_data_2_addr),
  .pub_app_data_2_RD1(ros2_pub_app_data_2_rdata),

  .pub_app_data_3_CS1(ros2_pub_app_data_3_ce),
  .pub_app_data_3_AD1(ros2_pub_app_data_3_addr),
  .pub_app_data_3_RD1(ros2_pub_app_data_3_rdata),
`endif

  .pub_app_data_len_0_rreq(),
  .pub_app_data_len_0_empty(1'b0),
  .pub_app_data_len_0_dout(ros2_pub_app_data_len_0),
  .pub_app_data_req_0_we(ros2_pub_app_data_ip_req[0]),
  .pub_app_data_req_0_wd(),
  .pub_app_data_rel_0_we(ros2_pub_app_data_ip_rel[0]),
  .pub_app_data_rel_0_wd(),
  .pub_app_data_grant_0_rd({7'd0, ros2_pub_app_data_ip_grant[0]}),

  .pub_app_data_len_1_rreq(),
  .pub_app_data_len_1_empty(1'b0),
  .pub_app_data_len_1_dout(ros2_pub_app_data_len_1),
  .pub_app_data_req_1_we(ros2_pub_app_data_ip_req[1]),
  .pub_app_data_req_1_wd(),
  .pub_app_data_rel_1_we(ros2_pub_app_data_ip_rel[1]),
  .pub_app_data_rel_1_wd(),
  .pub_app_data_grant_1_rd({7'd0, ros2_pub_app_data_ip_grant[1]}),

  .pub_app_data_len_2_rreq(),
  .pub_app_data_len_2_empty(1'b0),
  .pub_app_data_len_2_dout(ros2_pub_app_data_len_2),
  .pub_app_data_req_2_we(ros2_pub_app_data_ip_req[2]),
  .pub_app_data_req_2_wd(),
  .pub_app_data_rel_2_we(ros2_pub_app_data_ip_rel[2]),
  .pub_app_data_rel_2_wd(),
  .pub_app_data_grant_2_rd({7'd0, ros2_pub_app_data_ip_grant[2]}),

  .pub_app_data_len_3_rreq(),
  .pub_app_data_len_3_empty(1'b0),
  .pub_app_data_len_3_dout(ros2_pub_app_data_len_3),
  .pub_app_data_req_3_we(ros2_pub_app_data_ip_req[3]),
  .pub_app_data_req_3_wd(),
  .pub_app_data_rel_3_we(ros2_pub_app_data_ip_rel[3]),
  .pub_app_data_rel_3_wd(),
  .pub_app_data_grant_3_rd({7'd0, ros2_pub_app_data_ip_grant[3]})
);
`endif

endmodule

// arbiter for sharing app_data between user and IP
module app_data_arbiter (
    input wire i_clk,
    input wire i_rst_n,

    input wire i_en,

    input  wire i_app_data_ip_req,
    input  wire i_app_data_ip_rel,
    output wire o_app_data_ip_grant,

    input  wire i_app_data_user_req,
    input  wire i_app_data_user_rel,
    output wire o_app_data_user_ack,
    output wire o_app_data_user_nack,
    output wire o_app_data_user_grant
);
    localparam [1:0]
        APP_DATA_GRANT_NONE = 2'b00,
        APP_DATA_GRANT_IP   = 2'b01,
        APP_DATA_GRANT_USER = 2'b10;

    reg [1:0] r_app_data_grant;
    assign o_app_data_ip_grant = i_en & r_app_data_grant[0];
    assign o_app_data_user_grant = i_en & r_app_data_grant[1];

    reg r_user_ack, r_user_nack;
    reg r_last_user_req_sync;

    assign o_app_data_user_ack = r_user_ack | i_app_data_user_rel;
    assign o_app_data_user_nack = r_user_nack;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            r_app_data_grant <= APP_DATA_GRANT_NONE;
            r_user_ack <= 0;
            r_user_nack <= 0;
            r_last_user_req_sync <= 0;
        end else begin
            r_last_user_req_sync <= i_app_data_user_req;

            case (r_app_data_grant)
                APP_DATA_GRANT_NONE: begin
                    case ({i_app_data_ip_req, i_app_data_user_req})
                        2'b00: r_app_data_grant <= APP_DATA_GRANT_NONE;
                        2'b01: {r_app_data_grant, r_user_ack} <= {APP_DATA_GRANT_USER, 1'b1};
                        2'b10: r_app_data_grant <= APP_DATA_GRANT_IP;
                        2'b11: {r_app_data_grant, r_user_nack} <= {APP_DATA_GRANT_IP, 1'b1};
                    endcase
                end
                APP_DATA_GRANT_IP:
                    if (i_app_data_ip_rel) r_app_data_grant <= APP_DATA_GRANT_NONE;
                APP_DATA_GRANT_USER:
                    if (i_app_data_user_rel) r_app_data_grant <= APP_DATA_GRANT_NONE;
                default:
                    r_app_data_grant <= APP_DATA_GRANT_NONE;
            endcase

            if (r_last_user_req_sync & ~i_app_data_user_req) begin
                r_user_ack <= 0;
                r_user_nack <= 0;
            end
        end
    end
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
