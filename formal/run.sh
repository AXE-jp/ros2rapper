#!/bin/bash

# Copyright (c) 2021-2024 AXE, Inc.
# SPDX-License-Identifier: GPL-3.0-or-later

FV_CASES=`find *.sby -maxdepth 0`

for sbyfile in $FV_CASES; do
  sby -f $sbyfile
  if [ $? -ne 0 ]; then
    exit 1
  fi
done
