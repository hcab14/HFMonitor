#!/bin/bash
# $Id$

source functions.sh
    
(   trap "rm -f ${LOCK_FILE}; exit" EXIT
    flock -n -e 200 || { echo "This script $0 is currently being run"; exit 1; } >&2
    status=`check_running_all`
    [ X`last_status` != X$status ] && echo `date +%Y-%m-%dT%H:%M:%S` $status >> ${LOG_STATUS}
) 200>${LOCK_FILE}
