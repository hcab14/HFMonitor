#!/bin/bash
# $Id$

function check_client {
    if [ -f .pid_client ]; then
	echo YES
    else
	echo NO
    fi
}
function check_server {
    if [ `check_client` == YES ] || [ -f .pid_server ]; then
	echo YES
    else
	echo NO
    fi
}

function restart_client {    
    ./stop.sh client > /dev/null
    sleep 5s
    ./start.sh client > /dev/null
    echo FAIL_client
}
function restart_server {
    ./stop.sh server > /dev/null
    sleep 10s
    ./start.sh server > /dev/null
    sleep 30s
    echo FAIL_server
}
function restart_client_server {
    ./stop.sh client > /dev/null
    sleep 5s
    ./stop.sh server > /dev/null
    sleep 10s
    ./start.sh server > /dev/null
    sleep 30s
    ./start.sh client > /dev/null
    echo FAIL_server_client
}

function check_running {    
    local status_client=`./status.sh client` 
    local status_server=`./status.sh server`
    
    if [ `check_client` == YES ]; then 
	[ ${status_client} == RUN ] && [ ${status_server} == RUN ] && echo OK
	[ ${status_client} != RUN ] && [ ${status_server} == RUN ] && restart_client
	[ ${status_client} == RUN ] && [ ${status_server} != RUN ] && restart_client_server
	[ ${status_client} != RUN ] && [ ${status_server} != RUN ] && restart_client_server
    else	
	if [ `check_server` == YES ]; then 
	    [ ${status_server} == RUN ] && echo OK
	    [ ${status_server} == RUN ] && restart_server
	else 
	    echo OFF
	fi
    fi
}

function check_for_locks {
    ls .lock_* > /dev/null 2>&1
    [ $? == 0 ] && echo LOCKED || echo FREE
}

LOG_STATUS=log_status.txt

function last_status {
    if [ -f ${LOG_STATUS} ]; then
	tail -1 ${LOG_STATUS} | awk '{print $2}'
    else
	echo FAIL
    fi
}

LOCK_FILE=/tmp/HFMonitor_new.check_running.lockfile
(   
    flock -n -e 200 || { echo "This script is currently being run"; exit 1; } >&2
    if [ `check_for_locks` == FREE ]; then
	result=`check_running`
	echo result=$result last_status=`last_status`
	
	if [[ `last_status` != OK ]] || [[ $result != OK ]]; then
	    echo `date +%Y-%m-%dT%H:%M:%S` $result >> ${LOG_STATUS}
	fi
    fi
) 200>${LOCK_FILE}


