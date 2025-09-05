# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

.PHONY: create-proj hls_pub hls_sub synth clean

create-proj: hls_pub hls_sub
	$(MAKE) -C ../../src/ros2rapper -f Makefile.vitis
	vivado -mode batch -source create_project.tcl

hls_pub: hls_pub/solution1/impl/export.zip
hls_sub: hls_sub/solution1/impl/export.zip

%/solution1/impl/export.zip: hls/%.tcl hls/%.cpp
	vitis_hls -f $<

synth:
	vivado -mode batch -source synth_impl.tcl

clean:
	rm -f *.jou *.log *.xpr
	rm -rf ros2rapper-1KB hls_pub hls_sub .Xil
