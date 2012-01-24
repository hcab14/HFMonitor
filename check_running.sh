#!/bin/bash
# $Id$

function check_running {    
    local status_client=`./status.sh client` 
    local status_server=`./status.sh server`

    if [ ${status_client} == RUN ]; then
	if [ ${status_server} == RUN ]; then
	    echo OK
	else
	    ./stop.sh client
	    sleep 10s
	    ./start.sh server
	    sleep 30s
	    ./start.sh client
	    echo FAIL_server
	fi
    else
	if [ ${status_server} == RUN ]; then
	    ./start.sh client
	    echo FAIL_client
	else
	    ./start.sh server
	    sleep 30s
	    ./start.sh client
	    echo FAIL_server_client	
	fi
    fi
}

LOG_STATUS=log_status.txt

function last_status {
    if [ -f ${LOG_STATUS} ]; then
	tail -1 ${LOG_STATUS} | awk '{print $2}'
    else
	echo FAIL
    fi
}

result=`check_running`

echo result=$result last_status=`last_status`

if [[ `last_status` != OK ]] || [[ $result != OK ]]; then
    echo `date +%Y-%m-%dT%H:%M:%S` $result >> ${LOG_STATUS}
fi