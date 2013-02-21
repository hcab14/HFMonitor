#!/bin/bash
# $Id$

function check_run {
    local pid=`ps -p $1 -opid --no-heading`
    echo $(( $pid ))
}

function stop {
    local what=$1
    local pid_file=.pid_$what
    
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
		rm -f $pid_file
	    fi
	else
	    echo "$what ($pid) not running"
	    rm -f $pid_file
	fi
    else
	echo "$pid_file does not exist"
    fi    
}

LOCK_FILE=/tmp/HFMonitor_new.lockfile
(   
    flock -n -e 200 || { echo "This script is currently being run"; exit 1; } >&2
    if [ $# == 0 ]; then
	stop client
	stop server
    else
	stop $1
    fi
    
) 200>${LOCK_FILE}
