# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

SHELL := /bin/bash

FV_CASES = $(shell find *.sby -maxdepth 0)

.PHONY: all
all: test

.PHONY: clean
clean:
	find . -mindepth 1 -maxdepth 1 -type d -exec rm -r {} \;

.PHONY: test
test: clean
	 ./run.sh
