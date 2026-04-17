# Copyright (c) 2021-2026 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

.PHONY: create-vitis-proj create-cwb-proj synth clean

create-vitis-proj:
	$(MAKE) -C ../../src/ros2rapper -f Makefile.vitis
	vivado -mode batch -source create_project.tcl -tclargs vitis

create-cwb-proj:
	vivado -mode batch -source create_project.tcl -tclargs cwb

synth:
	vivado -mode batch -source synth_impl.tcl

clean:
	rm -f *.jou *.log *.xpr
	rm -rf ros2rapper-multi-topic-sub
