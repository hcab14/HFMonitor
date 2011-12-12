#!/bin/bash
# $Id$

function check_run {
    ps -p $1 -opid --no-heading
}

what=$1
pid_file=.pid_$what

if [ -f $pid_file ]; then
    pid=`cat $pid_file`
    if [ X$pid == X`check_run $pid` ]; then
	sudo kill $pid
	sleep 1s
        ## check if stop worked
	if [ X$pid == X`check_run $pid` ]; then
	    echo "$what ($pid) stop failed. Process is still running"
	else
	    echo "$what ($pid) stopped"
	fi
    else
	echo "$what ($pid) not running"
    fi
else
    echo "$pid_file does not exist"
fi