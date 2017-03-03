#!/bin/bash

P=$(dirname $0)

LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$P/software/install/lib \
    $P/software/install/bin/volk-config-info --cflags | \
    tail -1 | \
    awk -F::: '//{print $3}'
