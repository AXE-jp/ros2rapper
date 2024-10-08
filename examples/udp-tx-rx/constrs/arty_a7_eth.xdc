# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

set_property -dict {LOC E3 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.000 -name clk [get_ports clk]

set_property -dict {LOC C2 IOSTANDARD LVCMOS33} [get_ports rst_n]

set_false_path -from [get_ports rst_n]
set_input_delay 0.000 [get_ports rst_n]

set_property -dict {LOC F15 IOSTANDARD LVCMOS33} [get_ports phy_rx_clk]
set_property -dict {LOC D18 IOSTANDARD LVCMOS33} [get_ports {phy_rxd[0]}]
set_property -dict {LOC E17 IOSTANDARD LVCMOS33} [get_ports {phy_rxd[1]}]
set_property -dict {LOC E18 IOSTANDARD LVCMOS33} [get_ports {phy_rxd[2]}]
set_property -dict {LOC G17 IOSTANDARD LVCMOS33} [get_ports {phy_rxd[3]}]
set_property -dict {LOC G16 IOSTANDARD LVCMOS33} [get_ports phy_rx_dv]
set_property -dict {LOC C17 IOSTANDARD LVCMOS33} [get_ports phy_rx_er]
set_property -dict {LOC H16 IOSTANDARD LVCMOS33} [get_ports phy_tx_clk]
set_property -dict {LOC H14 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {phy_txd[0]}]
set_property -dict {LOC J14 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {phy_txd[1]}]
set_property -dict {LOC J13 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {phy_txd[2]}]
set_property -dict {LOC H17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports {phy_txd[3]}]
set_property -dict {LOC H15 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports phy_tx_en]
set_property -dict {LOC G18 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports phy_ref_clk]
set_property -dict {LOC C16 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports phy_rst_n]

create_clock -period 40.000 -name phy_rx_clk [get_ports phy_rx_clk]
create_clock -period 40.000 -name phy_tx_clk [get_ports phy_tx_clk]

set_property -dict {PACKAGE_PIN H5 IOSTANDARD LVCMOS33} [get_ports led4]
set_property -dict {PACKAGE_PIN J5 IOSTANDARD LVCMOS33} [get_ports led5]
set_property -dict {PACKAGE_PIN T9 IOSTANDARD LVCMOS33} [get_ports led6]
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports led7]

## Switches
#set_property -dict {PACKAGE_PIN A8 IOSTANDARD LVCMOS33} [get_ports sw0]
#set_property -dict {PACKAGE_PIN C11 IOSTANDARD LVCMOS33} [get_ports sw1]
#set_property -dict {PACKAGE_PIN C10 IOSTANDARD LVCMOS33} [get_ports sw2]
#set_property -dict {PACKAGE_PIN A10 IOSTANDARD LVCMOS33} [get_ports sw3]

set_false_path -to [get_ports {phy_ref_clk phy_rst_n}]
set_output_delay 0.000 [get_ports {phy_ref_clk phy_rst_n}]

set_clock_groups -asynchronous -group {clk_int} -group {phy_rx_clk}
set_clock_groups -asynchronous -group {clk_int} -group {phy_tx_clk}
