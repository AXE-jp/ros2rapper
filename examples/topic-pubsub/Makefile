# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

.PHONY: create-proj synth clean

create-proj:
	$(MAKE) -C ../src/ros2rapper vitis
	vivado -mode batch -source create_project.tcl

synth:
	vivado -mode batch -source synth_impl.tcl

clean:
	rm -f *.jou *.log *.xpr
	rm -rf ros2rapper-pubsub
