#!/usr/bin/env ruby

# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

SRCDIR = ARGV[0]

# remove zero delay (#0) timing control for Verilator (causes SEGV)
Dir.glob(SRCDIR+'/*.v') do |filename|
  fixed = File.read(filename)
    .gsub(/\/\/ power-on initialization(.*?)^end$/m, "")
  File.write(filename, fixed)
end
