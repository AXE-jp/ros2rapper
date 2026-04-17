// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`timescale 1ns / 1ps
`default_nettype none

`define ROS2_MAX_RAW_ETH_RX_DATA_LEN 1600

module test_raw_eth_rx_adapter();

    logic [47:0] src_mac = 48'hcb_a9_87_65_43_21;
    logic [47:0] dest_mac = 48'hff_ff_ff_ff_ff_ff;

    logic [15:0] eth_type_arp = {8'h06, 8'h08};
    logic [15:0] eth_type_ipv4 = {8'h00, 8'h08};
    logic [15:0] eth_type_another = {8'hff, 8'hff};

    logic [31:0] ip_addr = {8'd2, 8'd1, 8'd168, 8'd192};
    logic [31:0] subnet_mask = 32'h00_ff_ff_ff;
    logic [31:0] src_ip_addr = {8'd4, 8'd1, 8'd168, 8'd192};
    logic [31:0] ip_addr_unicast_to_another = {8'd3, 8'd1, 8'd168, 8'd192};
    logic [31:0] ip_addr_multicast = {8'd1, 8'd0, 8'd255, 8'd239};
    logic [31:0] ip_addr_direct_broadcast_to_me = {8'd255, 8'd1, 8'd168, 8'd192};
    logic [31:0] ip_addr_direct_broadcast_to_others = {8'd255, 8'd2, 8'd168, 8'd192};
    logic [31:0] ip_addr_limited_broadcast = 32'hff_ff_ff_ff;

    logic [7:0] ip_protocol_udp = 8'd17;
    logic [7:0] ip_protocol_another = 8'hff;

    logic [15:0] src_port = 16'h34_12;
    logic [15:0] dest_port = 16'h78_56;

    logic [127:0] dummy_payload = 128'h11_ff_ee_dd_cc_bb_aa_99_88_77_66_55_44_33_22_11;

    localparam [15:0] ARP_PACKET_LEN = 16'd30;
    logic [239:0] arp_packet = {dummy_payload, eth_type_arp, src_mac, dest_mac};

    localparam [15:0] ANOTHER_ETH_PACKET_LEN = 16'd30;
    logic [239:0] another_eth_packet = {dummy_payload, eth_type_another, src_mac, dest_mac};

    localparam [15:0] IPV4_ANOTHER_PACKET_LEN = 16'd50;
    logic [399:0] ipv4_another_packet_unicast_to_me = {
        dummy_payload,
        // IPv4 header
        ip_addr, src_ip_addr,
        16'd0, // checksum
        ip_protocol_another, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd36, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [399:0] ipv4_another_packet_unicast_to_another = {
        dummy_payload,
        // IPv4 header
        ip_addr_unicast_to_another, src_ip_addr,
        16'd0, // checksum
        ip_protocol_another, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd36, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [399:0] ipv4_another_packet_multicast = {
        dummy_payload,
        // IPv4 header
        ip_addr_multicast, src_ip_addr,
        16'd0, // checksum
        ip_protocol_another, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd36, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [399:0] ipv4_another_packet_direct_broadcast_to_me = {
        dummy_payload,
        // IPv4 header
        ip_addr_direct_broadcast_to_me, src_ip_addr,
        16'd0, // checksum
        ip_protocol_another, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd36, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [399:0] ipv4_another_packet_direct_broadcast_to_others = {
        dummy_payload,
        // IPv4 header
        ip_addr_direct_broadcast_to_others, src_ip_addr,
        16'd0, // checksum
        ip_protocol_another, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd36, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [399:0] ipv4_another_packet_limited_broadcast = {
        dummy_payload,
        // IPv4 header
        ip_addr_limited_broadcast, src_ip_addr,
        16'd0, // checksum
        ip_protocol_another, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd36, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    localparam [15:0] UDP_PACKET_LEN = 16'd54;
    logic [431:0] udp_packet = {
        dummy_payload,
        // UDP header
        16'd0, // UDP checksum
        8'd24, 8'd0, // payload length
        dest_port, src_port,
        // IPv4 header
        ip_addr, src_ip_addr,
        16'd0, // checksum
        ip_protocol_udp, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd44, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    localparam [15:0] RTPS_PACKET_LEN = 16'd58;
    logic [463:0] rtps_packet = {
        dummy_payload,
        "SPTR",
        // UDP header
        16'd0, // UDP checksum
        8'd28, 8'd0, // payload length
        dest_port, src_port,
        // IPv4 header
        ip_addr, src_ip_addr,
        16'd0, // checksum
        ip_protocol_udp, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd48, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [463:0] rtps_packet_with_another_eth_type = {
        dummy_payload,
        "SPTR",
        // UDP header
        16'd0, // UDP checksum
        8'd28, 8'd0, // payload length
        dest_port, src_port,
        // IPv4 header
        ip_addr, src_ip_addr,
        16'd0, // checksum
        ip_protocol_udp, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd48, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_another, src_mac, dest_mac};

    logic [463:0] rtps_packet_with_more_fragment = {
        dummy_payload,
        "SPTR",
        // UDP header
        16'd0, // UDP checksum
        8'd28, 8'd0, // payload length
        dest_port, src_port,
        // IPv4 header
        ip_addr, src_ip_addr,
        16'd0, // checksum
        ip_protocol_udp, 8'd64, // Time to live
        8'h00, 8'h20, // flags and fragmentation offset
        16'd0, // identification
        8'd48, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [463:0] rtps_packet_with_fragment_offset = {
        dummy_payload,
        "SPTR",
        // UDP header
        16'd0, // UDP checksum
        8'd28, 8'd0, // payload length
        dest_port, src_port,
        // IPv4 header
        ip_addr, src_ip_addr,
        16'd0, // checksum
        ip_protocol_udp, 8'd64, // Time to live
        8'hff, 8'h1f, // flags and fragmentation offset
        16'd0, // identification
        8'd48, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [463:0] rtps_packet_with_another_protocol = {
        dummy_payload,
        "SPTR",
        // UDP header
        16'd0, // UDP checksum
        8'd28, 8'd0, // payload length
        dest_port, src_port,
        // IPv4 header
        ip_addr, src_ip_addr,
        16'd0, // checksum
        ip_protocol_another, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd48, 8'd0, // total length
        8'h00, 8'h45,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    localparam [15:0] RTPS_PACKET_WITH_OPTIONS_LEN = 16'd62;
    logic [495:0] rtps_packet_with_options = {
        dummy_payload,
        "SPTR",
        // UDP header
        16'd0, // UDP checksum
        8'd28, 8'd0, // payload length
        dest_port, src_port,
        // IPv4 header
        32'd0, // padding
        ip_addr, src_ip_addr,
        16'd0, // checksum
        ip_protocol_udp, 8'd64, // Time to live
        8'h00, 8'h40, // flags and fragmentation offset
        16'd0, // identification
        8'd52, 8'd0, // total length
        8'h00, 8'h46,
        // ether frame header
        eth_type_ipv4, src_mac, dest_mac};

    logic [31:0] error_code;

    logic clk;
    logic rst_n;
    localparam DELAY = 1;
    always begin
        clk = 1'b1;
        #5;
        clk = 1'b0;
        #5;
    end

    localparam ADDR_WIDTH = $clog2(`ROS2_MAX_RAW_ETH_RX_DATA_LEN) - 2;
    logic [ADDR_WIDTH-1:0] ram_addr;
    logic ram_ce;
    logic [3:0]  ram_we;
    logic [31:0] ram_wdata;
    logic [31:0] ram [0:(`ROS2_MAX_RAW_ETH_RX_DATA_LEN/4)-1];

    task automatic reset_ram();
        integer i;
        for (i = 0; i < (`ROS2_MAX_RAW_ETH_RX_DATA_LEN/4); i = i + 1) begin
            ram[i] = 32'd0;
        end
    endtask

    task automatic ram_is_not_touched();
        integer i;
        for (i = 0; i < (`ROS2_MAX_RAW_ETH_RX_DATA_LEN/4); i = i + 1) begin
            assert (ram[i] == 32'd0) else begin
                error_code = 32'd1;
                $finish;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_ram();
        end else begin
            if (ram_ce) begin
                if (ram_we[0]) ram[ram_addr][7:0]   = ram_wdata[7:0];
                if (ram_we[1]) ram[ram_addr][15:8]  = ram_wdata[15:8];
                if (ram_we[2]) ram[ram_addr][23:16] = ram_wdata[23:16];
                if (ram_we[3]) ram[ram_addr][31:24] = ram_wdata[31:24];
            end
        end
    end

    logic frame_ready;
    logic ack;

    logic [7:0] axis_tdata;
    logic axis_tvalid;
    logic axis_tready;
    logic axis_tlast;

    raw_eth_rx_adapter
    raw_eth_rx_adapter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(1'b1),
        .rx_raw_eth_axis_tdata(axis_tdata),
        .rx_raw_eth_axis_tvalid(axis_tvalid),
        .rx_raw_eth_axis_tready(axis_tready),
        .rx_raw_eth_axis_tlast(axis_tlast),
        .rx_raw_eth_data_addr(ram_addr),
        .rx_raw_eth_data_ce(ram_ce),
        .rx_raw_eth_data_we(ram_we),
        .rx_raw_eth_data_wdata(ram_wdata),
        .rx_raw_eth_data_frame_ready(frame_ready),
        .rx_raw_eth_data_ack(ack),
        .ip_addr(ip_addr),
        .subnet_mask(subnet_mask)
    );

    localparam MAX_PACKET_LEN = 62;
    task automatic send_packet(
        input [15:0] packet_len,
        input [MAX_PACKET_LEN*8-1:0] packet
    );
        integer j;
        #(DELAY);
        axis_tdata  = 8'd0;
        axis_tvalid = 1'b0;
        axis_tlast  = 1'b0;

        for (j = 0; j < packet_len - 1; j = j + 1) begin
            #(DELAY);
            axis_tdata  = packet[8*j +: 8];
            axis_tvalid = 1'b1;
            while (!axis_tready) begin
                @(posedge clk);
                #(DELAY);
            end
            @(posedge clk);
        end

        #(DELAY);
        axis_tdata  = packet[8*packet_len-8 +: 8];
        axis_tvalid = 1'b1;
        axis_tlast  = 1'b1;
        while (!axis_tready) begin
            @(posedge clk);
            #(DELAY);
        end
        @(posedge clk);
        #(DELAY);
        axis_tdata  = 8'd0;
        axis_tvalid = 1'b0;
        axis_tlast  = 1'b0;
    endtask

    task automatic send_packet_with_interval(
        input [15:0] packet_len,
        input [MAX_PACKET_LEN*8-1:0] packet
    );
        integer j;
        #(DELAY);
        axis_tdata  = 8'd0;
        axis_tvalid = 1'b0;
        axis_tlast  = 1'b0;

        for (j = 0; j < packet_len - 1; j = j + 1) begin
            #(DELAY);
            axis_tdata  = packet[8*j +: 8];
            axis_tvalid = 1'b1;
            while (!axis_tready) begin
                @(posedge clk);
                #(DELAY);
            end
            @(posedge clk);
            #(DELAY);
            axis_tdata  = 8'd0;
            axis_tvalid = 1'b0;
            repeat(3) @(posedge clk);
        end

        #(DELAY);
        axis_tdata  = packet[8*packet_len-8 +: 8];
        axis_tvalid = 1'b1;
        axis_tlast  = 1'b1;
        while (!axis_tready) begin
            @(posedge clk);
            #(DELAY);
        end
        @(posedge clk);
        #(DELAY);
        axis_tdata  = 8'd0;
        axis_tvalid = 1'b0;
        axis_tlast  = 1'b0;
    endtask

    task automatic check_ram(
        input [15:0] packet_len,
        input [MAX_PACKET_LEN*8-1:0] packet
    );
        logic [ADDR_WIDTH+1:0] index;
        logic [ADDR_WIDTH-1:0] addr;
        logic [4:0] offset;
        integer j;
        assert (ram[0][15:0] == packet_len) else begin
            error_code = 32'd2;
            $finish;
        end
        for (j = 0; j < packet_len; j = j + 1) begin
            index  = j + 2;
            addr   = index[ADDR_WIDTH+1:2];
            offset = 8 * index[1:0];
            assert (ram[addr][offset +: 8] == packet[8*j +: 8]) else begin
                error_code = 32'd3;
                $finish;
            end
        end
    endtask

    task automatic test_packet_ignored (
        input [15:0] packet_len,
        input [MAX_PACKET_LEN*8-1:0] packet
    );
        #(DELAY);
        reset_ram();
        ack = 1'b0;
        @(posedge clk);
        send_packet(.packet_len(packet_len), .packet(packet));
        repeat(20) @(posedge clk);
        assert (frame_ready == 1'b0) else begin
            error_code = 32'd4;
            $finish;
        end
        ram_is_not_touched();
        send_packet_with_interval(.packet_len(packet_len), .packet(packet));
        repeat(20) @(posedge clk);
        assert (frame_ready == 1'b0) else begin
            error_code = 32'd5;
            $finish;
        end
        ram_is_not_touched();
    endtask

    task automatic test_packet_received (
        input [15:0] packet_len,
        input [MAX_PACKET_LEN*8-1:0] packet
    );
        #(DELAY);
        ack = 1'b0;
        @(posedge clk);
        send_packet(.packet_len(packet_len), .packet(packet));
        repeat(20) @(posedge clk);
        assert (frame_ready == 1'b1) else begin
            error_code = 32'd6;
            $finish;
        end
        check_ram(.packet_len(packet_len), .packet(packet));
        #(DELAY);
        ack = 1'b1;
        while (frame_ready) begin
            @(posedge clk);
            #(DELAY);
        end
        reset_ram();
        ack = 1'b0;
        @(posedge clk);
        send_packet_with_interval(.packet_len(packet_len), .packet(packet));
        repeat(20) @(posedge clk);
        assert (frame_ready == 1'b1) else begin
            error_code = 32'd7;
            $finish;
        end
        check_ram(.packet_len(packet_len), .packet(packet));
        #(DELAY);
        ack = 1'b1;
        while (frame_ready) begin
            @(posedge clk);
            #(DELAY);
        end
        ack = 1'b0;
        @(posedge clk);
    endtask

    initial begin
        error_code = 32'd0;
        rst_n = 1'b0;
        repeat(10) @(posedge clk);
        rst_n = 1'b1;
        repeat(10) @(posedge clk);

        $display("ARP");
        test_packet_ignored(.packet_len(ARP_PACKET_LEN), .packet(arp_packet));
        $display("Another Ether packet");
        test_packet_received(.packet_len(ANOTHER_ETH_PACKET_LEN), .packet(another_eth_packet));

        $display("IPv4 Unicast to me");
        test_packet_received(.packet_len(IPV4_ANOTHER_PACKET_LEN), .packet(ipv4_another_packet_unicast_to_me));
        $display("IPv4 Unicast to another");
        test_packet_ignored(.packet_len(IPV4_ANOTHER_PACKET_LEN), .packet(ipv4_another_packet_unicast_to_another));
        $display("IPv4 Multicast");
        test_packet_received(.packet_len(IPV4_ANOTHER_PACKET_LEN), .packet(ipv4_another_packet_multicast));
        $display("IPv4 Direct Broadcast to me");
        test_packet_received(.packet_len(IPV4_ANOTHER_PACKET_LEN), .packet(ipv4_another_packet_direct_broadcast_to_me));
        $display("IPv4 Direct Broadcast to others");
        test_packet_ignored(.packet_len(IPV4_ANOTHER_PACKET_LEN), .packet(ipv4_another_packet_direct_broadcast_to_others));
        $display("IPv4 Limited Broadcast");
        test_packet_received(.packet_len(IPV4_ANOTHER_PACKET_LEN), .packet(ipv4_another_packet_limited_broadcast));

        $display("UDP");
        test_packet_received(.packet_len(UDP_PACKET_LEN), .packet(udp_packet));
        $display("RTPS");
        test_packet_ignored(.packet_len(RTPS_PACKET_LEN), .packet(rtps_packet));
        $display("RTPS with another ether frame type");
        test_packet_received(.packet_len(RTPS_PACKET_LEN), .packet(rtps_packet_with_another_eth_type));
        $display("RTPS with more fragment");
        test_packet_received(.packet_len(RTPS_PACKET_LEN), .packet(rtps_packet_with_more_fragment));
        $display("RTPS with fragment offset");
        test_packet_received(.packet_len(RTPS_PACKET_LEN), .packet(rtps_packet_with_fragment_offset));
        $display("RTPS with another protocol");
        test_packet_received(.packet_len(RTPS_PACKET_LEN), .packet(rtps_packet_with_another_protocol));
        $display("RTPS with IPv4 header options");
        test_packet_received(.packet_len(RTPS_PACKET_WITH_OPTIONS_LEN), .packet(rtps_packet_with_options));

        // Test whether the packet is ignored when frame_ready == 1'b1
        send_packet(.packet_len(IPV4_ANOTHER_PACKET_LEN), .packet(ipv4_another_packet_unicast_to_me));
        repeat(20) @(posedge clk);
        assert (frame_ready == 1'b1) else begin
            error_code = 32'd8;
            $finish;
        end
        reset_ram();
        send_packet(.packet_len(IPV4_ANOTHER_PACKET_LEN), .packet(ipv4_another_packet_unicast_to_me));
        repeat(20) @(posedge clk);
        ram_is_not_touched();

        $finish;
    end
 endmodule

`resetall
