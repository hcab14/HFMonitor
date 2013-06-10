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
    local pid=$((`cat $pid_file | tail -1`))
    [ X$pid == X0 ]     && { echo STOPPED; return; }
    [ X$pid == X$((`ps -p $pid -opid --no-heading`)) ] && echo $pid || echo STOPPED;
}

function stop {
    local name=$1
    local pid=`check_running $name`
    [ $pid == STOPPED ] && { rm -f .pid_$name; return; }
    sudo kill $pid
    sleep 10s
    pid=`check_running $name`
    [ $pid == STOPPED ] && { rm -f .pid_$name; return; }
    sudo kill -9 $pid
    sleep 5s
    pid=`check_running $name`
    [ $pid == STOPPED ] && { rm -f .pid_$name; return; }
    echo UNABLE TO STOP $name > /dev/stderr
}

function where_am_I {
    local username=`whoami`
    case $username in
	gifds-hv)
	    echo HTV
	    ;;
	gifds-nz)
	    echo NTZ
	    ;;
	chm)
	    echo KRK
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
    for i in `config_data`; do
	source $i
	[ `check_running $NAME` == STOPPED ] && { start $NAME $CMD; sleep 5s; }
	if [ `check_running $NAME` == STOPPED ]; then
	    echo "## start $NAME failed: check for error messages in ./Log and in log_$NAME.txt:"
	    cat log_$NAME.txt
	    break;
	fi	
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
    local result_stopped=""
    for i in `config_data`; do
	source $i
        # when there is no .pid file we consider the process to be stopped and do not restart it
	[ ! -f .pid_$NAME ] && { result_stopped="${result_stopped},$NAME"; continue; }
	[ `check_running $NAME` == STOPPED ] && { start $NAME $CMD; sleep 5s; result="${result},$NAME"; }
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
