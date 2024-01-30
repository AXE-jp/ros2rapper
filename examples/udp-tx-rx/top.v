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

    // --- Clock & Reset
    wire clk_int;
    wire clk_25mhz_int;
    wire rst_n_int;
    wire mmcm_locked;
    wire mmcm_clkfb;

    assign phy_ref_clk = clk_25mhz_int;

    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKOUT0_DIVIDE_F(10),
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0),
        .CLKOUT1_DIVIDE(40),
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
        .CLKFBOUT_MULT_F(10),
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

    // --- ARP Configuration
    localparam ARP_REQUEST_RETRY_COUNT = 4;
    localparam ARP_REQUEST_RETRY_INTERVAL = (125000000*2);
    localparam ARP_REQUEST_TIMEOUT = (125000000*30);

    // --- UDP Configuration
    wire [15:0] ros2_rx_udp_port = 16'd1234;

    // --- UDP RX Control
    reg rxbuf_rel;
    wire rxbuf_grant;

    always @(posedge clk_int or negedge rst_n_int) begin
        if (!rst_n_int) begin
            rxbuf_rel <= 0;
        end else begin
            if (rxbuf_grant)
                rxbuf_rel <= 1;
            else
                rxbuf_rel <= 0;
        end
    end

    // --- UDP RX Buffer
    wire [`UDP_RXBUF_AWIDTH-1:0] rxbuf_addr;
    wire rxbuf_ce;
    wire rxbuf_we;
    wire [31:0] rxbuf_wdata;
    reg [31:0] rx_buf[0:63];

    always @(posedge clk_int or negedge rst_n_int) begin
        if (!rst_n_int) begin
        end else begin
            if (rxbuf_we)
                rx_buf[rxbuf_addr] <= rxbuf_wdata;
        end
    end

    // Indicate received UDP payload size by LED
    wire [15:0] rx_payload_len = rx_buf[1][31:16] - 8;
    assign led4 = (rx_payload_len >= 1);
    assign led5 = (rx_payload_len >= 5);
    assign led6 = (rx_payload_len >= 10);
    assign led7 = (rx_payload_len >= 15);

    // --- UDP TX Control
    reg txbuf_rel;
    wire txbuf_grant;
    reg [27:0] tx_counter;

    always @(posedge clk_int or negedge rst_n_int) begin
        if (!rst_n_int) begin
            txbuf_rel <= 0;
            tx_counter <= 0;
        end else begin
            if (tx_counter[27]) begin
                txbuf_rel <= 1;
                tx_counter <= 0;
            end else begin
                txbuf_rel <= 0;
                tx_counter <= tx_counter + 1;
            end
        end
    end

    // --- UDP TX Buffer
    reg [31:0] txbuf_rdata;
    wire [`UDP_TXBUF_AWIDTH-1:0] txbuf_addr;
    always @(*) begin
        case (txbuf_addr)
        6'h00: txbuf_rdata <= 32'h0a01a8c0;  // Destinaton IP address: 192.168.1.10
        6'h01: txbuf_rdata <= 32'h0457_04d2; // Source Port: 1111, Destination Port: 1234
        6'h02: txbuf_rdata <= 32'h00000007;  // Payload length: 7
        6'h03: txbuf_rdata <= 32'h20504455;  // Payload: "UDP Send Test\n"
        6'h04: txbuf_rdata <= 32'h646e6553;
        6'h05: txbuf_rdata <= 32'h73655420;
        6'h06: txbuf_rdata <= 32'h00000074;
        default: txbuf_rdata <= 32'h0;
        endcase
    end

    // --- ROS2rapper with Ethernet
    ros2_ether ros2 (
        .clk(clk_int),
        .rst_n(rst_n_int),

        .ether_en(1'b1),
        .ros2pub_en(1'b0),
        .ros2sub_en(1'b0),

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

        .ros2_node_name(0),
        .ros2_node_name_len(0),
        .ros2_node_udp_port(0),
        .ros2_rx_udp_port(ros2_rx_udp_port),
        .ros2_port_num_seed(0),
        .ros2_tx_period(0),
        .ros2_fragment_expiration(0),
        .ros2_guid_prefix(0),

        .ros2_pub_topic_name(0),
        .ros2_pub_topic_name_len(0),
        .ros2_pub_topic_type_name(0),
        .ros2_pub_topic_type_name_len(0),

        .ros2_sub_topic_name(0),
        .ros2_sub_topic_name_len(0),
        .ros2_sub_topic_type_name(0),
        .ros2_sub_topic_type_name_len(0),

        .ros2_pub_app_data(),
        .ros2_pub_app_data_len(),
        .ros2_pub_app_data_req(1'b0),
        .ros2_pub_app_data_rel(1'b0),
        .ros2_pub_app_data_grant(),

        .ros2_sub_app_data_addr(),
        .ros2_sub_app_data_ce(),
        .ros2_sub_app_data_we(),
        .ros2_sub_app_data_wdata(),
        .ros2_sub_app_data_len(),
        .ros2_sub_app_data_req(1'b0),
        .ros2_sub_app_data_rel(1'b0),
        .ros2_sub_app_data_grant(),
        .ros2_sub_app_data_recv(),

        .udp_rxbuf_rel(rxbuf_rel),
        .udp_rxbuf_grant(rxbuf_grant),
        .udp_rxbuf_addr(rxbuf_addr),
        .udp_rxbuf_ce(rxbuf_ce),
        .udp_rxbuf_we(rxbuf_we),
        .udp_rxbuf_wdata(rxbuf_wdata),

        .udp_txbuf_grant(txbuf_grant),
        .udp_txbuf_rel(txbuf_rel),
        .udp_txbuf_addr(txbuf_addr),
        .udp_txbuf_ce(),
        .udp_txbuf_rdata(txbuf_rdata),

        .payloadsmem_addr(),
        .payloadsmem_ce(),
        .payloadsmem_we(),
        .payloadsmem_wdata(),
        .payloadsmem_rdata(),

        .arp_req_retry_count(ARP_REQUEST_RETRY_COUNT),
        .arp_req_retry_interval(ARP_REQUEST_RETRY_INTERVAL),
        .arp_req_timeout(ARP_REQUEST_TIMEOUT)
    );

endmodule

`resetall
