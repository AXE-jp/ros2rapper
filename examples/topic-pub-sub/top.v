// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none

`include "ros2_config.vh"
`include "ros2_ether_config.vh"

module top (
    input  wire       clk,
    input  wire       rst_n,

    output wire       phy_ref_clk,
    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_dv,
    input  wire       phy_rx_er,
    input  wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_en,
    output wire       phy_rst_n,

    output wire       led4,
    output wire       led5,
    output wire       led6,
    output wire       led7
);

    localparam ARP_REQUEST_RETRY_COUNT = 4;
    localparam ARP_REQUEST_RETRY_INTERVAL = (125000000*2);
    localparam ARP_REQUEST_TIMEOUT = (125000000*30);

    // --- Clock & Reset
    wire clk_int;
    wire clk_25mhz_int;
    wire rst_n_int;
    wire mmcm_locked;
    wire mmcm_clkfb;

    assign phy_ref_clk = clk_25mhz_int;

    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKOUT0_DIVIDE_F(13),
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0),
        .CLKOUT1_DIVIDE(39),
        .CLKOUT1_DUTY_CYCLE(0.5),
        .CLKOUT1_PHASE(0),
        .CLKOUT2_DIVIDE(1),
        .CLKOUT2_DUTY_CYCLE(0.5),
        .CLKOUT2_PHASE(0),
        .CLKOUT3_DIVIDE(1),
        .CLKOUT3_DUTY_CYCLE(0.5),
        .CLKOUT3_PHASE(0),
        .CLKOUT4_DIVIDE(1),
        .CLKOUT4_DUTY_CYCLE(0.5),
        .CLKOUT4_PHASE(0),
        .CLKOUT5_DIVIDE(1),
        .CLKOUT5_DUTY_CYCLE(0.5),
        .CLKOUT5_PHASE(0),
        .CLKOUT6_DIVIDE(1),
        .CLKOUT6_DUTY_CYCLE(0.5),
        .CLKOUT6_PHASE(0),
        .CLKFBOUT_MULT_F(9.75),
        .CLKFBOUT_PHASE(0),
        .DIVCLK_DIVIDE(1),
        .REF_JITTER1(0.010),
        .CLKIN1_PERIOD(10.0),
        .STARTUP_WAIT("FALSE"),
        .CLKOUT4_CASCADE("FALSE")
    )
    clk_mmcm_inst (
        .CLKIN1(clk),
        .CLKFBIN(mmcm_clkfb),
        .RST(~rst_n),
        .PWRDWN(1'b0),
        .CLKOUT0(clk_int),
        .CLKOUT0B(),
        .CLKOUT1(clk_25mhz_int),
        .CLKOUT1B(),
        .CLKOUT2(),
        .CLKOUT2B(),
        .CLKOUT3(),
        .CLKOUT3B(),
        .CLKOUT4(),
        .CLKOUT5(),
        .CLKOUT6(),
        .CLKFBOUT(mmcm_clkfb),
        .CLKFBOUTB(),
        .LOCKED(mmcm_locked)
    );

    reg [3:0] sync_rst_reg;
    assign rst_n_int = sync_rst_reg[3];

    always @(posedge clk_int or negedge rst_n) begin
        if (!rst_n) begin
            sync_rst_reg <= 0;
        end else begin
            sync_rst_reg <= {sync_rst_reg[2:0], mmcm_locked};
        end
    end

    // --- Ethernet Configuration
    wire [47:0] mac_addr         = 48'h00_00_00_00_00_02;
    wire [31:0] ip_addr          = {8'd100, 8'd1, 8'd168, 8'd192};
    wire [31:0] gateway_ip_addr  = {8'd1, 8'd1, 8'd168, 8'd192};
    wire [31:0] subnet_mask      = {8'd0, 8'd255, 8'd255, 8'd255};

    // --- ROS2 Node Configuration
    wire [`ROS2_MAX_NODE_NAME_LEN*8-1:0] ros2_node_name = "reklat";
    wire [7:0] ros2_node_name_len = 8'd7;
    wire [15:0] ros2_node_udp_port = 16'd52000;
    wire [15:0] ros2_cpu_udp_port = 16'd1234;
    wire [15:0] ros2_port_num_seed = 16'd7400;
    wire [31:0] ros2_tx_period = 32'd12500000;
    wire [31:0] ros2_fragment_expiration = 32'd3333333333;
    wire [95:0] ros2_guid_prefix = 96'h00_00_00_01_00_00_09_de_ad_37_0f_01;

    // --- ROS2 Pubisher Configuration
    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_pub_topic_name = "rettahc/tr";
    wire [7:0] ros2_pub_topic_name_len = 8'd11;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_pub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_pub_topic_type_name_len = 8'd29;
    wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] ros2_app_data = "!dlrow reppar2SOR ,olleh"; // Published message
    wire [7:0] ros2_app_data_len = 8'd25;

    // --- ROS2 Subscriber Configuration
    wire [`ROS2_MAX_TOPIC_NAME_LEN*8-1:0] ros2_sub_topic_name = "rettahc/tr";
    wire [7:0] ros2_sub_topic_name_len = 8'd11;
    wire [`ROS2_MAX_TOPIC_TYPE_NAME_LEN*8-1:0] ros2_sub_topic_type_name = "_gnirtS::_sdd::gsm::sgsm_dts";
    wire [7:0] ros2_sub_topic_type_name_len = 8'd29;

    // --- ROS2 Subscriber Received Message
    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_app_rx_data_addr;
    wire ros2_app_rx_data_ce;
    wire ros2_app_rx_data_we;
    wire [7:0] ros2_app_rx_data_wdata;
    reg [7:0] rx_msg_reg[0:`ROS2_MAX_APP_DATA_LEN-1];
    always @(posedge clk_int) begin
        if (ros2_app_rx_data_ce & ros2_app_rx_data_we)
            rx_msg_reg[ros2_app_rx_data_addr][7:0] <= ros2_app_rx_data_wdata;
    end
    wire [7:0] ros2_app_rx_data_len;
    assign led4 = rx_msg_reg[0][0];
    assign led5 = rx_msg_reg[0][1];
    assign led6 = rx_msg_reg[0][2];
    assign led7 = rx_msg_reg[0][3];

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
        .i_clk(clk_int),
        .i_rst_n(rst_n_int),
        .i_cs_n(~payloadsmem_cs),
        .i_we_n(~payloadsmem_we),
        .i_wmask(4'b1111),
        .i_addr(payloadsmem_addr),
        .i_wdata(payloadsmem_wdata),
        .o_rdata(payloadsmem_rdata)
    );

    // --- ROS2rapper with Ethernet
    ros2_ether ros2 (
        .clk(clk_int),
        .rst_n(rst_n_int),

        .ether_en(1'b1),
        .ros2pub_en(1'b1),
        .ros2sub_en(1'b1),

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
        .ros2_cpu_udp_port(ros2_cpu_udp_port),
        .ros2_port_num_seed(ros2_port_num_seed),
        .ros2_tx_period(ros2_tx_period),
        .ros2_fragment_expiration(ros2_fragment_expiration),
        .ros2_guid_prefix(ros2_guid_prefix),

        .ros2_pub_topic_name(ros2_pub_topic_name),
        .ros2_pub_topic_name_len(ros2_pub_topic_name_len),
        .ros2_pub_topic_type_name(ros2_pub_topic_type_name),
        .ros2_pub_topic_type_name_len(ros2_pub_topic_type_name_len),

        .ros2_sub_topic_name(ros2_sub_topic_name),
        .ros2_sub_topic_name_len(ros2_sub_topic_name_len),
        .ros2_sub_topic_type_name(ros2_sub_topic_type_name),
        .ros2_sub_topic_type_name_len(ros2_sub_topic_type_name_len),

        .ros2_app_data(ros2_app_data),
        .ros2_app_data_len(ros2_app_data_len),
        .ros2_app_data_cpu_req(1'b0),
        .ros2_app_data_cpu_rel(1'b0),
        .ros2_app_data_cpu_grant(),

        .ros2_app_rx_data_addr(ros2_app_rx_data_addr),
        .ros2_app_rx_data_ce(ros2_app_rx_data_ce),
        .ros2_app_rx_data_we(ros2_app_rx_data_we),
        .ros2_app_rx_data_wdata(ros2_app_rx_data_wdata),
        .ros2_app_rx_data_len(ros2_app_rx_data_len),
        .ros2_app_rx_data_cpu_req(1'b0),
        .ros2_app_rx_data_cpu_rel(1'b0),
        .ros2_app_rx_data_cpu_grant(),

        .udp_rxbuf_cpu_rel(1'b0),
        .udp_rxbuf_cpu_grant(),
        .udp_rxbuf_addr(),
        .udp_rxbuf_ce(),
        .udp_rxbuf_we(),
        .udp_rxbuf_wdata(),

        .udp_txbuf_cpu_rel(1'b0),
        .udp_txbuf_cpu_grant(),
        .udp_txbuf_addr(),
        .udp_txbuf_ce(),
        .udp_txbuf_rdata(32'b0),

        .payloadsmem_addr(payloadsmem_addr),
        .payloadsmem_ce(payloadsmem_cs),
        .payloadsmem_we(payloadsmem_we),
        .payloadsmem_wdata(payloadsmem_wdata),
        .payloadsmem_rdata(payloadsmem_rdata),

        .arp_req_retry_count(ARP_REQUEST_RETRY_COUNT),
        .arp_req_retry_interval(ARP_REQUEST_RETRY_INTERVAL),
        .arp_req_timeout(ARP_REQUEST_TIMEOUT)
    );

endmodule

`resetall
