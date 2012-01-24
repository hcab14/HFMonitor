#!/bin/bash
# $Id$

function check_run {
    local pid=`ps -p $1 -opid --no-heading`
    echo $(( $pid ))
}

pid_file=.pid_$1
if [ -f $pid_file ]; then
    pid=`cat $pid_file`
    [ X$pid == X`check_run $pid` ] && echo RUN || echo STOPPED
else
    echo STOPPED
fi
