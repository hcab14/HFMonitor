#!/bin/bash
# $Id$

if [ `./status.sh server` != RUN ]; then
    ./stop.sh client
    ./start.sh server
    sleep 20s
    ./start.sh client
    sleep 5s
fi

if [ `./status.sh client` != RUN ]; then
    ./start.sh client
fi
