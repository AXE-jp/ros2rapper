// Copyright (c) 2021-2026 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none
`timescale 1ns / 1ps

module test_axis_async_fifo();
    logic s_clk;
    logic s_rst_n;
    logic s_drop;
    logic [7:0] s_axis_tdata;
    logic s_axis_tvalid;
    logic s_axis_tready;
    logic s_axis_tlast;
    logic s_axis_tuser;
    logic m_clk;
    logic m_rst_n;
    logic [7:0] m_axis_tdata;
    logic m_axis_tvalid;
    logic m_axis_tready;
    logic m_axis_tlast;
    logic m_axis_tuser;
    logic s_status_overflow;
    logic s_status_bad_frame;
    logic s_status_good_frame;
    logic m_status_overflow;
    logic m_status_bad_frame;
    logic m_status_good_frame;

    axis_async_fifo #(
        .DEPTH(2048),
        .DATA_WIDTH(8),
        .KEEP_ENABLE(0),
        .KEEP_WIDTH(1),
        .LAST_ENABLE(1),
        .ID_ENABLE(0),
        .ID_WIDTH(8),
        .DEST_ENABLE(0),
        .DEST_WIDTH(8),
        .USER_ENABLE(1),
        .USER_WIDTH(1),
        .PIPELINE_OUTPUT(2),
        .FRAME_FIFO(1),
        .USER_BAD_FRAME_VALUE(1'b1),
        .USER_BAD_FRAME_MASK(1'b1),
        .DROP_OVERSIZE_FRAME(1),
        .DROP_BAD_FRAME(1),
        .DROP_WHEN_FULL(1)
    )
    axis_async_fifo_inst (
        .s_clk(s_clk),
        .s_rst_n(s_rst_n),
        .s_drop(s_drop),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tkeep(1'b0),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .s_axis_tlast(s_axis_tlast),
        .s_axis_tid(8'd0),
        .s_axis_tdest(8'd0),
        .s_axis_tuser(s_axis_tuser),
        .m_clk(m_clk),
        .m_rst_n(m_rst_n),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tkeep(),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast),
        .m_axis_tid(),
        .m_axis_tdest(),
        .m_axis_tuser(m_axis_tuser),
        .s_status_overflow(s_status_overflow),
        .s_status_bad_frame(s_status_bad_frame),
        .s_status_good_frame(s_status_good_frame),
        .m_status_overflow(m_status_overflow),
        .m_status_bad_frame(m_status_bad_frame),
        .m_status_good_frame(m_status_good_frame)
    );

    // s_clk @ 25 MHz
    always begin
        s_clk = 1'b1;
        #20;
        s_clk = 1'b0;
        #20;
    end
    // m_clk @ 160 MHz
    always begin
        m_clk = 1'b1;
        #3.125;
        m_clk = 1'b0;
        #3.125;
    end

    localparam DELAY = 1;
    logic [31:0] error_code;

    task automatic send_frame(
        input [31:0] length
    );
        logic [31:0] i;
        for (i = 0; i < length; i = i + 1) begin
            @(posedge s_clk);
            #(DELAY);
            s_axis_tdata = i[7:0];
            s_axis_tvalid = 1'b1;
            s_axis_tlast = (i + 1 == length);
            s_axis_tuser = 1'b0;
            while (!s_axis_tready) @(posedge s_clk);
        end
        @(posedge s_clk);
        #(DELAY);
        s_axis_tdata = 8'd0;
        s_axis_tvalid = 1'b0;
        s_axis_tlast = 1'b0;
        s_axis_tuser = 1'b0;
    endtask

    task automatic check_frame(
        input [31:0] length
    );
        logic [31:0] i;
        for (i = 0; i < length; i = i + 1) begin
            @(posedge m_clk);
            #(DELAY);
            m_axis_tready = 1'b1;
            while (!m_axis_tvalid) begin
                @(posedge m_clk);
                #(DELAY);
            end
            assert (m_axis_tdata == i[7:0]) else begin
                error_code = 32'd1;
                $finish;
            end
            assert (m_axis_tlast == (i + 1 == length)) else begin
                error_code = 32'd2;
                $finish;
            end
            assert (m_axis_tuser == 1'b0) else begin
                error_code = 32'd3;
                $finish;
            end
        end
        @(posedge m_clk);
        #(DELAY);
        m_axis_tready = 1'b0;
    endtask

    task automatic check_s_status(
        input overflow,
        input bad_frame,
        input good_frame
    );
        while (!s_status_overflow && !s_status_bad_frame && !s_status_good_frame) begin
            @(posedge s_clk);
            #(DELAY);
        end
        assert (s_status_overflow == overflow) else begin
            error_code = 32'd4;
            $finish;
        end
        assert (s_status_bad_frame == bad_frame) else begin
            error_code = 32'd5;
            $finish;
        end
        assert (s_status_good_frame == good_frame) else begin
            error_code = 32'd6;
            $finish;
        end
    endtask

    task automatic check_m_status(
        input overflow,
        input bad_frame,
        input good_frame
    );
        while (!m_status_overflow && !m_status_bad_frame && !m_status_good_frame) begin
            @(posedge m_clk);
            #(DELAY);
        end
        assert (m_status_overflow == overflow) else begin
            error_code = 32'd7;
            $finish;
        end
        assert (m_status_bad_frame == bad_frame) else begin
            error_code = 32'd8;
            $finish;
        end
        assert (m_status_good_frame == good_frame) else begin
            error_code = 32'd9;
            $finish;
        end
    endtask

    initial begin
        // Initialize
        error_code = 32'd0;
        s_rst_n = 1'b0;
        s_drop = 1'b1;
        s_axis_tdata = 8'd0;
        s_axis_tvalid = 1'b0;
        s_axis_tlast = 1'b0;
        s_axis_tuser = 1'b0;
        m_rst_n = 1'b0;
        m_axis_tready = 1'b0;
        #1000;
        s_rst_n = 1'b1;
        m_rst_n = 1'b1;
        #1000;
        // Test with s_drop==1'b1
        send_frame(.length(2048));
        check_s_status(.overflow(1'b1), .bad_frame(1'b0), .good_frame(1'b0));
        check_m_status(.overflow(1'b1), .bad_frame(1'b0), .good_frame(1'b0));
        send_frame(.length(1));
        check_s_status(.overflow(1'b1), .bad_frame(1'b0), .good_frame(1'b0));
        check_m_status(.overflow(1'b1), .bad_frame(1'b0), .good_frame(1'b0));
        // Set s_drop 1'b0
        @(posedge s_clk);
        #(DELAY);
        s_drop = 1'b0;
        // Test with s_drop==1'b0
        send_frame(.length(2048));
        check_s_status(.overflow(1'b0), .bad_frame(1'b0), .good_frame(1'b1));
        check_m_status(.overflow(1'b0), .bad_frame(1'b0), .good_frame(1'b1));
        check_frame(.length(2048));
        send_frame(.length(1));
        check_s_status(.overflow(1'b0), .bad_frame(1'b0), .good_frame(1'b1));
        check_m_status(.overflow(1'b0), .bad_frame(1'b0), .good_frame(1'b1));
        check_frame(.length(1));
        $finish;
    end
endmodule

`resetall
