// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none

`include "ros2_config.vh"

`define ROS2CLK_HZ 50000000

module ros2rapper_tx_counters #
(
    parameter PRESCALER_DIV               = 16,
    parameter TX_INTERVAL_COUNT           = (`ROS2CLK_HZ / PRESCALER_DIV) / 100,
    parameter TX_PERIOD_SPDP_WR_COUNT     = `ROS2CLK_HZ / PRESCALER_DIV,
    parameter TX_PERIOD_SEDP_PUB_WR_COUNT = `ROS2CLK_HZ / PRESCALER_DIV,
    parameter TX_PERIOD_SEDP_SUB_WR_COUNT = `ROS2CLK_HZ / PRESCALER_DIV,
    parameter TX_PERIOD_SEDP_PUB_HB_COUNT = `ROS2CLK_HZ / PRESCALER_DIV,
    parameter TX_PERIOD_SEDP_SUB_HB_COUNT = `ROS2CLK_HZ / PRESCALER_DIV,
    parameter TX_PERIOD_SEDP_PUB_AN_COUNT = `ROS2CLK_HZ / PRESCALER_DIV,
    parameter TX_PERIOD_SEDP_SUB_AN_COUNT = `ROS2CLK_HZ / PRESCALER_DIV,
    parameter TX_PERIOD_APP_WR_COUNT      = `ROS2CLK_HZ / PRESCALER_DIV
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
    reg [$clog2(PRESCALER_DIV              )-1:0] cnt_prescaler;
    reg [$clog2(TX_INTERVAL_COUNT          )-1:0] cnt_interval;
    reg [$clog2(TX_PERIOD_SPDP_WR_COUNT    )-1:0] cnt_spdp_wr;
    reg [$clog2(TX_PERIOD_SEDP_PUB_WR_COUNT)-1:0] cnt_sedp_pub_wr;
    reg [$clog2(TX_PERIOD_SEDP_SUB_WR_COUNT)-1:0] cnt_sedp_sub_wr;
    reg [$clog2(TX_PERIOD_SEDP_PUB_HB_COUNT)-1:0] cnt_sedp_pub_hb;
    reg [$clog2(TX_PERIOD_SEDP_SUB_HB_COUNT)-1:0] cnt_sedp_sub_hb;
    reg [$clog2(TX_PERIOD_SEDP_PUB_AN_COUNT)-1:0] cnt_sedp_pub_an;
    reg [$clog2(TX_PERIOD_SEDP_SUB_AN_COUNT)-1:0] cnt_sedp_sub_an;
    reg [$clog2(TX_PERIOD_APP_WR_COUNT     )-1:0] cnt_app_wr;

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
