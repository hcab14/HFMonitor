#!/bin/bash
# $Id$

function check_run {
    local pid=`ps -p $1 -opid --no-heading`
    echo $(( $pid ))
}

lock_file=.lock_stop_$$
touch ${lock_file} && trap "rm -f ${lock_file}" EXIT || ( echo LOCK_FAILED > /dev/stderr; exit 1 )

if [ $# == 0 ]; then
    $0 client
    $0 server
else

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
		rm -f $pid_file
	    fi
	else
	    echo "$what ($pid) not running"
	    rm -f $pid_file
	fi
    else
	echo "$pid_file does not exist"
    fi    
fi
