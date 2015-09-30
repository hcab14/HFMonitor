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
#    at now <<EOF
#date >> log_$name.txt
#$cmd >> log_$name.txt 2>&1 &
#echo \$! > .pid_$name
#sleep 1s##
#
#EOF

    date >> log_$name.txt
    nohup $cmd >> log_$name.txt 2>&1 200>&- &
    echo $! > .pid_$name
    sleep 1s
}

## returns 
##  * "STOPPED": process was properly stopped
##  * "DEAD":    process died
##  * the pid of the running process
function check_running {
    local name=$1
    local pid_file=.pid_$name
    [ ! -f $pid_file ] && { echo STOPPED; return; }
    local pid=$((`cat $pid_file | tail -1`))
    [ X$pid == X0 ]     && { echo DEAD; return; }
    [ X$pid == X$((`ps -p $pid -opid --no-heading`)) ] && echo $pid || echo DEAD;
}

function stop {
    local name=$1
    local pid=`check_running $name`
    [ $pid == DEAD ] && { rm -f .pid_$name; return; }
    sudo kill $pid
    sleep 10s
    pid=`check_running $name`
    [ $pid == DEAD ] && { rm -f .pid_$name; return; }
    sudo kill -9 $pid
    sleep 5s
    pid=`check_running $name`
    [ $pid == DEAD ] && { rm -f .pid_$name; return; }
    echo UNABLE TO STOP $name > /dev/stderr
}

function where_am_I {
    local username=`whoami`
    case $username in
	gifds-hv)
	    echo BOS
	    ;;
	gifds-nz)
	    echo NTZ
	    ;;
	chm)
	    echo KRK
	    ;;
	vlfmonitor)
	    echo MUN
	    ;;
	gifds-st)
	    echo STA
	    ;;
	*)
	    echo XXX
	    ;;
    esac
}

function config_data {
    find run/`where_am_I` -name "*.in" -print | sort
}

## -----------------------------------------------------------------------------

function start_all {
    mkdir -p Log
    for i in `config_data`; do
	source $i
	local status=`check_running $NAME`
	if [ $status == STOPPED ] || [ $status == DEAD ]; then
	    start $NAME $CMD
	    sleep 5s
	fi
	status=`check_running $NAME`
	if [ $status == STOPPED ] || [ $status == DEAD ]; then
	    echo "## start $NAME failed: check for error messages in ./Log and in log_$NAME.txt:"
	    tail -10 log_$NAME.txt
	fi
    done
}

function stop_all {
    for i in `config_data`; do
	source $i
	local status=`check_running $NAME`
	if [ ! $status == STOPPED ]; then 
	    stop $NAME
	    [ ! $status == DEAD ] && sleep 5s
	fi
    done
}

function check_running_all {
    local result=""
    local result_stopped=""
    for i in `config_data`; do
	source $i
	local status=`check_running $NAME`
	[ $status == STOPPED ] && { result_stopped="${result_stopped},$NAME"; continue; }
	[ $status == DEAD ]    && { start $NAME $CMD; sleep 5s; result="${result},$NAME"; }
    done

    if [ X$result == X ] && [ X$result_stopped == X ]; then
	echo OK;
    else 
	local prefix_fail=""
	local prefix_stopped=""
	[ X$result != X ]         && prefix_fail=FAIL:
	[ X$result_stopped != X ] && prefix_stopped=STOPPED:
	echo ${prefix_fail}${result:1}${prefix_stopped}${result_stopped:1}
    fi
}
