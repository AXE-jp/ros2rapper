#!/bin/bash

SUBS=$(find . -mindepth 1 -maxdepth 1 -type d | grep -v '\-cwb')

for dir in $SUBS; do
    make -C $dir create-proj synth

    if [ $? != 0 ]; then
        echo "Error in $dir"
        exit 1
    else
        echo "Success in $dir"
    fi
done
