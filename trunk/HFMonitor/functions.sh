#!/bin/bash
# $Id$

LOG_STATUS="log_status.txt"
LOCK_FILE="/tmp/HFMonitor_new.lock"

trap "exit" SIGINT SIGQUIT SIGTERM

function last_status {
    [ -f ${LOG_STATUS} ] && { tail -1 ${LOG_STATUS} | awk '{print $2}'; } || echo FAIL;
}

function start {
    local name=$1; shift
    local cmd=$@;
    local pid_file=.pid_$name    
    at now <<EOF
$cmd > log_$name.txt 2>&1 &
echo \$! > .pid_$name
EOF
}

## returns "STOPPED" or the pid of the running process
function check_running {
    local name=$1
    local pid_file=.pid_$name
    [ ! -f $pid_file ] && { echo STOPPED; return; }
    local pid=`cat $pid_file | tail -1`
    [ X$pid == X ]     && { echo STOPPED; return; }
    [ X$pid == X$((`ps -p $pid -opid --no-heading`)) ] && echo $pid || echo STOPPED;
}

function stop {
    local name=$1
    local pid=`check_running $name`
    [ $pid == STOPPED ] && { rm -f .pid_$name; return; }
    kill $pid
    sleep 10s
    pid=`check_running $name`
    [ $pid == STOPPED ] && { rm -f .pid_$name; return; }
    kill -9 $pid
    sleep 5s
    pid=`check_running $name`
    [ $pid == STOPPED ] && { rm -f .pid_$name; return; }
    echo UNABLE TO STOP $name > /dev/stderr
}

function where_am_I {
    [ X`hostname` == X"nz23-18.ifj.edu.pl" ] && echo KRK || echo NTZ
}
function config_data {
    find run/`where_am_I` -name "*.in" -print | sort
}

## -----------------------------------------------------------------------------

function start_all {
    for i in `config_data`; do
	source $i
	[ `check_running $NAME` == STOPPED ] && { start $NAME $CMD; sleep 5s; }
    done
}

function stop_all {
    for i in `config_data | tac`; do
	source $i
	[ ! `check_running $NAME` == STOPPED ] && { stop $NAME; sleep 5s; }
    done
}

function check_running_all {
    local result=""
    for i in `config_data`; do
	source $i
	[ ! -f .pid_$name ] && continue;
	[ `check_running $NAME` == STOPPED ] && { start $NAME $CMD; sleep 5s; result="${result}_$name"; }
    done
    [ X$result == X ] && echo OK || echo FAIL$result
}
