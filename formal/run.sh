#!/bin/bash

FV_CASES=`find *.sby -maxdepth 0`

for sbyfile in $FV_CASES; do
  sby -f $sbyfile
  if [ $? -ne 0 ]; then
    exit 1
  fi
done
