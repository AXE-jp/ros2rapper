#!/bin/bash

# Fix read but never set variables (Spyglass List W123)

declare -a vars=("RG_694" "RG_695" "RG_697" "RG_698" "RG_699" "RG_700" "RG_701" "RG_702" "RG_724" "RG_725" "RG_726" "RG_727" "RG_728" "RG_729" "RG_730" "RG_731" "RG_732" "RG_733" "RG_743")
#declare -a multibit_vars=()

file=$1

for var in "${vars[@]}"; do
  sed -i "s/reg\s$var\s;/wire $var = 0;/g" $file
done

#for var in "${multibit_vars[@]}"; do
#  sed -i "s/reg\s\(\[.*\]\)\s$var\s;/wire \1 $var = 0;/g" $file
#done
