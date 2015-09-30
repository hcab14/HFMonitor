#!/bin/bash
# $Id$

source functions.sh

(   flock -n -e 200 || { echo "This script $0 is currently locked"; exit 1; } >&2
    status=`check_running_all`
    echo $status
    [ X`last_status` != X$status ] && echo `date +%Y-%m-%dT%H:%M:%S` $status >> ${LOG_STATUS}
) 200>${LOCK_FILE}
