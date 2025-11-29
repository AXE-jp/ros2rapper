// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none

`include "ros2_config.vh"
`include "ros2_ether_config.vh"

module ros2_module #(
    parameter ROS2CLK_HZ = 100_000_000
)
(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       clk_25mhz,

    output wire       phy_ref_clk,
    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_dv,
    input  wire       phy_rx_er,
    input  wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_en,
    output wire       phy_rst_n,

    output reg        led4,
    output reg        led5,

    output reg  ros2_pub_app_data_ap_start,
    input  wire ros2_pub_app_data_ap_ready,
    input  wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-2:0] ros2_pub_app_data_addr0,
    input  wire ros2_pub_app_data_ce0,
    input  wire ros2_pub_app_data_we0,
    input  wire [15:0] ros2_pub_app_data_wdata0,
    output reg  [15:0] pub_data_seed,

    output wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_addr,
    output wire ros2_sub_app_data_ce,
    output wire ros2_sub_app_data_we,
    output wire [7:0] ros2_sub_app_data_wdata,

    input  wire sub_data_result,
    input  wire sub_data_result_ap_vld,
    output wire sub_data_result_ap_ack
);

    assign phy_ref_clk = clk_25mhz;

    // --- Ethernet Configuration
    wire [47:0] mac_addr         = 48'h00_00_00_00_00_02;
    wire [31:0] ip_addr          = {8'd100, 8'd1, 8'd168, 8'd192};
    wire [31:0] gateway_ip_addr  = {8'd1, 8'd1, 8'd168, 8'd192};
    wire [31:0] subnet_mask      = {8'd0, 8'd255, 8'd255, 8'd255};

    // --- ARP Configuration
    localparam ARP_REQUEST_RETRY_COUNT = 4;
    localparam ARP_REQUEST_RETRY_INTERVAL = (125000000*2);
    localparam ARP_REQUEST_TIMEOUT = (125000000*30);

    // --- ROS2 Node Configuration
    wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name = "elpmaxe_reppar2sor/";
    wire [7:0] ros2_node_name_len = 8'd22;
    wire [15:0] ros2_node_udp_port = 16'd52000;
    wire [15:0] ros2_port_num_seed = 16'd7400;
    wire [31:0] ros2_fragment_expiration = 32'd3333333333;
    wire [95:0] ros2_guid_prefix = 96'h00_00_00_01_00_00_09_de_ad_37_0f_01;
    wire [31:0] ros2_participant_lease_duration_seconds = 32'd20;
    wire [31:0] ros2_participant_lease_duration_fraction = 32'd0;

    // --- ROS2 Pubisher Configuration
    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name = "cipot_elpmas/tr";
    wire [7:0] ros2_pub_topic_name_len = 8'd16;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name = "_215x61tniU::_sdd::gsm::sgsm_elpmas";
    wire [7:0] ros2_pub_topic_type_name_len = 8'd36;

    localparam [`ROS2_APP_DATA_LEN_WIDTH:0] ROS2_PUB_APP_DATA_LEN = 16'd1024;

`ifdef ROS2_PUB_DATA_FF
    reg [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_pub_app_data;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ros2_pub_app_data <= {`ROS2_MAX_APP_DATA_LEN{8'd0}};
        end else begin
            if (ros2_pub_app_data_ce0 & ros2_pub_app_data_we0) begin
                ros2_pub_app_data[16*ros2_pub_app_data_addr0 +: 16] <= ros2_pub_app_data_wdata0;
            end
        end
    end
`endif
`ifdef ROS2_PUB_DATA_RAM
    reg  [31:0] ros2_pub_app_data [0:`ROS2_MAX_APP_DATA_LEN/4-1];
    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_addr1;
    reg  [31:0] ros2_pub_app_data_rdata1;

    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-3:0] ros2_pub_app_data_addr0_32 = ros2_pub_app_data_addr0[$clog2(`ROS2_MAX_APP_DATA_LEN)-2:1];
    always @(posedge clk) begin
        if (!rst_n) begin
            ros2_pub_app_data_rdata1 <= 32'd0;
        end else begin
            ros2_pub_app_data_rdata1 <= ros2_pub_app_data[ros2_pub_app_data_addr1];
            if (ros2_pub_app_data_ce0 & ros2_pub_app_data_we0) begin
                if (ros2_pub_app_data_addr0[0]) begin
                    ros2_pub_app_data[ros2_pub_app_data_addr0_32][31:16] <= ros2_pub_app_data_wdata0;
                end else begin
                    ros2_pub_app_data[ros2_pub_app_data_addr0_32][15:0] <= ros2_pub_app_data_wdata0;
                end
            end
        end
    end
`endif

    // --- ROS2 Subscriber Configuration
    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name = "cipot_elpmas/tr";
    wire [7:0] ros2_sub_topic_name_len = 8'd16;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name = "_215x61tniU::_sdd::gsm::sgsm_elpmas";
    wire [7:0] ros2_sub_topic_type_name_len = 8'd36;

    localparam STATE_IDLE       = 0;
    localparam STATE_WAIT_GRANT = 1;
    localparam STATE_WAIT_VALID = 2;
    reg [$clog2(STATE_WAIT_VALID+1)-1:0] pub_state;
    reg [$clog2(STATE_WAIT_VALID+1)-1:0] sub_state;

    // --- ROS2 Publisher Message Control
    localparam COUNT_MAX = ROS2CLK_HZ - 1;
    reg [$clog2(COUNT_MAX+1)-1:0] count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= COUNT_MAX;
            pub_data_seed <= 16'd0;
        end else begin
            if (count == 0) begin
                count <= COUNT_MAX;
                pub_data_seed <= pub_data_seed + 1'b1;
            end else begin
                count <= count - 1'b1;
            end
        end
    end

    reg  ros2_pub_app_data_req_0;
    wire ros2_pub_app_data_rel_0;
    wire ros2_pub_app_data_ack_0;
    wire ros2_pub_app_data_nack_0;

    assign ros2_pub_app_data_rel_0 = (pub_state == STATE_WAIT_VALID) & ros2_pub_app_data_ap_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pub_state <= STATE_IDLE;
            ros2_pub_app_data_req_0 <= 1'b0;
            ros2_pub_app_data_ap_start <= 1'b0;
        end else begin
            if (pub_state == STATE_WAIT_GRANT) begin
                if (ros2_pub_app_data_ack_0) begin
                    pub_state <= STATE_WAIT_VALID;
                    ros2_pub_app_data_req_0 <= 1'b0;
                    ros2_pub_app_data_ap_start <= 1'b1;
                end else if (ros2_pub_app_data_nack_0) begin
                    pub_state <= STATE_IDLE;
                    ros2_pub_app_data_req_0 <= 1'b0;
                end
            end else if (pub_state == STATE_WAIT_VALID) begin
                if (ros2_pub_app_data_ap_ready) begin
                    pub_state <= STATE_IDLE;
                    ros2_pub_app_data_ap_start <= 1'b0;
                end
            end else begin  // pub_state == STATE_IDLE
                if (count == 0) begin
                    pub_state <= STATE_WAIT_GRANT;
                    ros2_pub_app_data_req_0 <= 1'b1;
                end
            end
        end
    end

    wire [3:0] ros2_pub_app_data_req;
    wire [3:0] ros2_pub_app_data_rel;
    wire [3:0] ros2_pub_app_data_ack;
    wire [3:0] ros2_pub_app_data_nack;

    assign ros2_pub_app_data_req[0] = ros2_pub_app_data_req_0;
    assign ros2_pub_app_data_rel[0] = ros2_pub_app_data_rel_0;
    assign ros2_pub_app_data_ack_0 = ros2_pub_app_data_ack[0];
    assign ros2_pub_app_data_nack_0 = ros2_pub_app_data_nack[0];

    // --- ROS2 Subscriber message control
    reg  ros2_sub_app_data_req_0;
    wire ros2_sub_app_data_rel_0;
    wire ros2_sub_app_data_ack_0;
    wire ros2_sub_app_data_nack_0;

    wire [63:0] recvinfo_din;
    wire recvinfo_write;

    assign sub_data_result_ap_ack = (sub_state == STATE_WAIT_GRANT) & ros2_sub_app_data_ack_0;
    assign ros2_sub_app_data_rel_0 = (sub_state == STATE_WAIT_VALID) & sub_data_result_ap_vld;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub_state <= STATE_IDLE;
            ros2_sub_app_data_req_0 <= 1'b0;
            led4 <= 1'b0;
            led5 <= 1'b0;
        end else begin
            if (sub_state == STATE_WAIT_GRANT) begin
                if (ros2_sub_app_data_ack_0) begin
                    sub_state <= STATE_WAIT_VALID;
                    ros2_sub_app_data_req_0 <= 1'b0;
                end else if (ros2_sub_app_data_nack_0) begin
                    sub_state <= STATE_IDLE;
                    ros2_sub_app_data_req_0 <= 1'b0;
                end
            end else if (sub_state == STATE_WAIT_VALID) begin
                if (sub_data_result_ap_vld) begin
                    sub_state <= STATE_IDLE;
                    led4 <= sub_data_result;
                    led5 <= ~sub_data_result;
                end
            end else begin  // sub_state == STATE_IDLE
                if (recvinfo_write && recvinfo_din[35:32] == 4'b0001) begin
                    sub_state <= STATE_WAIT_GRANT;
                    ros2_sub_app_data_req_0 <= 1'b1;
                end
            end
        end
    end

    wire [3:0] ros2_sub_app_data_req;
    wire [3:0] ros2_sub_app_data_rel;
    wire [3:0] ros2_sub_app_data_ack;
    wire [3:0] ros2_sub_app_data_nack;

    assign ros2_sub_app_data_req[0] = ros2_sub_app_data_req_0;
    assign ros2_sub_app_data_rel[0] = ros2_sub_app_data_rel_0;
    assign ros2_sub_app_data_ack_0 = ros2_sub_app_data_ack[0];
    assign ros2_sub_app_data_nack_0 = ros2_sub_app_data_nack[0];

    // --- SEDP Reader Table Memory
`ifdef ROS2_SEDP_READER_TBL_RAM
    wire [$clog2(`ROS2_SEDP_READER_MAX*11)-1:0] sedp_reader_tbl_mem_addr;
    wire sedp_reader_tbl_mem_cs;
    wire sedp_reader_tbl_mem_we;
    wire [63:0] sedp_reader_tbl_mem_wdata;
    wire [63:0] sedp_reader_tbl_mem_rdata;
    ram_1rw #(
        .DEPTH(`ROS2_SEDP_READER_MAX*11),
        .DWIDTH(64)
    )
    sedp_reader_tbl_mem (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_cs_n(~sedp_reader_tbl_mem_cs),
        .i_we_n(~sedp_reader_tbl_mem_we),
        .i_wmask(8'b11111111),
        .i_addr(sedp_reader_tbl_mem_addr),
        .i_wdata(sedp_reader_tbl_mem_wdata),
        .o_rdata(sedp_reader_tbl_mem_rdata)
    );
`endif

    // --- IP Payload Memory
    wire payloadsmem_cs;
    wire payloadsmem_we;
    wire [`PAYLOADSMEM_AWIDTH-1:0] payloadsmem_addr;
    wire [7:0] payloadsmem_wdata, payloadsmem_rdata;
    ram_1rw #(
        .DEPTH(`PAYLOADSMEM_DEPTH),
        .DWIDTH(8)
    )
    payloadsmem (
        .i_clk(clk),
        .i_rst_n(rst_n),
        .i_cs_n(~payloadsmem_cs),
        .i_we_n(~payloadsmem_we),
        .i_wmask(4'b1111),
        .i_addr(payloadsmem_addr),
        .i_wdata(payloadsmem_wdata),
        .o_rdata(payloadsmem_rdata)
    );

    // --- ROS2rapper with Ethernet
    localparam PRESCALER_DIV = 64;
    ros2_ether #(
        .SET_TX_PERIOD_BY_PARAMETER (1),
        .PRESCALER_DIV              (PRESCALER_DIV),
        .ROS2CLK_HZ                 (ROS2CLK_HZ),
        .TX_INTERVAL_COUNT          ((ROS2CLK_HZ / PRESCALER_DIV) / 100),
        .TX_PERIOD_SPDP_WR_COUNT    ((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_WR_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_WR_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_HB_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_HB_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_PUB_AN_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_SEDP_SUB_AN_COUNT((ROS2CLK_HZ / PRESCALER_DIV) * 3),
        .TX_PERIOD_APP_WR_COUNT     ((ROS2CLK_HZ / PRESCALER_DIV) * 3)
    )
    ros2 (
        .clk(clk),
        .rst_n(rst_n),

        .ether_en(1'b1),
        .ros2pub_en(1),
        .ros2sub_en(1),

        .phy_rx_clk(phy_rx_clk),
        .phy_rxd(phy_rxd),
        .phy_rx_dv(phy_rx_dv),
        .phy_rx_er(phy_rx_er),
        .phy_tx_clk(phy_tx_clk),
        .phy_txd(phy_txd),
        .phy_tx_en(phy_tx_en),
        .phy_rst_n(phy_rst_n),

        .mac_addr(mac_addr),
        .ip_addr(ip_addr),
        .gateway_ip_addr(gateway_ip_addr),
        .subnet_mask(subnet_mask),

        .ros2_node_name(ros2_node_name),
        .ros2_node_name_len(ros2_node_name_len),
        .ros2_node_udp_port(ros2_node_udp_port),
        .ros2_port_num_seed(ros2_port_num_seed),
        .ros2_fragment_expiration(ros2_fragment_expiration),
        .ros2_guid_prefix(ros2_guid_prefix),
        .ros2_participant_lease_duration_seconds(ros2_participant_lease_duration_seconds),
        .ros2_participant_lease_duration_fraction(ros2_participant_lease_duration_fraction),

        .ros2_pub_topic_name_0(ros2_pub_topic_name),
        .ros2_pub_topic_name_len_0(ros2_pub_topic_name_len),
        .ros2_pub_topic_type_name_0(ros2_pub_topic_type_name),
        .ros2_pub_topic_type_name_len_0(ros2_pub_topic_type_name_len),

        .ros2_pub_topic_name_1(0),
        .ros2_pub_topic_name_len_1(0),
        .ros2_pub_topic_type_name_1(0),
        .ros2_pub_topic_type_name_len_1(0),

        .ros2_pub_topic_name_2(0),
        .ros2_pub_topic_name_len_2(0),
        .ros2_pub_topic_type_name_2(0),
        .ros2_pub_topic_type_name_len_2(0),

        .ros2_pub_topic_name_3(0),
        .ros2_pub_topic_name_len_3(0),
        .ros2_pub_topic_type_name_3(0),
        .ros2_pub_topic_type_name_len_3(0),

        .ros2_sub_topic_name_0(ros2_sub_topic_name),
        .ros2_sub_topic_name_len_0(ros2_sub_topic_name_len),
        .ros2_sub_topic_type_name_0(ros2_sub_topic_type_name),
        .ros2_sub_topic_type_name_len_0(ros2_sub_topic_type_name_len),

        .ros2_sub_topic_name_1(0),
        .ros2_sub_topic_name_len_1(0),
        .ros2_sub_topic_type_name_1(0),
        .ros2_sub_topic_type_name_len_1(0),

        .ros2_sub_topic_name_2(0),
        .ros2_sub_topic_name_len_2(0),
        .ros2_sub_topic_type_name_2(0),
        .ros2_sub_topic_type_name_len_2(0),

        .ros2_sub_topic_name_3(0),
        .ros2_sub_topic_name_len_3(0),
        .ros2_sub_topic_type_name_3(0),
        .ros2_sub_topic_type_name_len_3(0),

`ifdef ROS2_PUB_DATA_FF
        .ros2_pub_app_data_0(ros2_pub_app_data),
        .ros2_pub_app_data_1(0),
        .ros2_pub_app_data_2(0),
        .ros2_pub_app_data_3(0),
`endif
`ifdef ROS2_PUB_DATA_RAM
        .ros2_pub_app_data_0_addr(ros2_pub_app_data_addr1),
        .ros2_pub_app_data_0_ce(),
        .ros2_pub_app_data_0_rdata(ros2_pub_app_data_rdata1),

        .ros2_pub_app_data_1_addr(),
        .ros2_pub_app_data_1_ce(),
        .ros2_pub_app_data_1_rdata(0),

        .ros2_pub_app_data_2_addr(),
        .ros2_pub_app_data_2_ce(),
        .ros2_pub_app_data_2_rdata(0),

        .ros2_pub_app_data_3_addr(),
        .ros2_pub_app_data_3_ce(),
        .ros2_pub_app_data_3_rdata(0),
`endif

        .ros2_pub_app_data_len_0(ROS2_PUB_APP_DATA_LEN),
        .ros2_pub_app_data_len_1(0),
        .ros2_pub_app_data_len_2(0),
        .ros2_pub_app_data_len_3(0),

        .ros2_pub_app_data_req(ros2_pub_app_data_req),
        .ros2_pub_app_data_rel(ros2_pub_app_data_rel),
        .ros2_pub_app_data_ack(ros2_pub_app_data_ack),
        .ros2_pub_app_data_nack(ros2_pub_app_data_nack),
        .ros2_pub_app_data_grant(),

        .ros2_sub_app_data_0_addr(ros2_sub_app_data_addr),
        .ros2_sub_app_data_0_ce(ros2_sub_app_data_ce),
        .ros2_sub_app_data_0_we(ros2_sub_app_data_we),
        .ros2_sub_app_data_0_wdata(ros2_sub_app_data_wdata),

        .ros2_sub_app_data_1_addr(),
        .ros2_sub_app_data_1_ce(),
        .ros2_sub_app_data_1_we(),
        .ros2_sub_app_data_1_wdata(),

        .ros2_sub_app_data_2_addr(),
        .ros2_sub_app_data_2_ce(),
        .ros2_sub_app_data_2_we(),
        .ros2_sub_app_data_2_wdata(),

        .ros2_sub_app_data_3_addr(),
        .ros2_sub_app_data_3_ce(),
        .ros2_sub_app_data_3_we(),
        .ros2_sub_app_data_3_wdata(),

        .ros2_sub_app_data_recvinfo_din(recvinfo_din),
        .ros2_sub_app_data_recvinfo_full_n(1'b1),
        .ros2_sub_app_data_recvinfo_write(recvinfo_write),

        .ros2_sub_app_data_req(ros2_sub_app_data_req),
        .ros2_sub_app_data_rel(ros2_sub_app_data_rel),
        .ros2_sub_app_data_ack(ros2_sub_app_data_ack),
        .ros2_sub_app_data_nack(ros2_sub_app_data_nack),
        .ros2_sub_app_data_grant(),

`ifdef ROS2_SEDP_READER_TBL_RAM
        .sedp_reader_tbl_mem_addr(sedp_reader_tbl_mem_addr),
        .sedp_reader_tbl_mem_ce(sedp_reader_tbl_mem_cs),
        .sedp_reader_tbl_mem_we(sedp_reader_tbl_mem_we),
        .sedp_reader_tbl_mem_wdata(sedp_reader_tbl_mem_wdata),
        .sedp_reader_tbl_mem_rdata(sedp_reader_tbl_mem_rdata),
`endif

        .ip_payloadsmem_addr(payloadsmem_addr),
        .ip_payloadsmem_ce(payloadsmem_cs),
        .ip_payloadsmem_we(payloadsmem_we),
        .ip_payloadsmem_wdata(payloadsmem_wdata),
        .ip_payloadsmem_rdata(payloadsmem_rdata),

        .arp_req_retry_count(ARP_REQUEST_RETRY_COUNT),
        .arp_req_retry_interval(ARP_REQUEST_RETRY_INTERVAL),
        .arp_req_timeout(ARP_REQUEST_TIMEOUT)
    );

endmodule

`resetall
