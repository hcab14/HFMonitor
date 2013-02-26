#!/bin/bash
# $Id$

source functions.sh

(   trap "rm -f ${LOCK_FILE}; exit" EXIT
    flock -n -e 200 || { echo "This script $0 is currently being run"; exit 1; } >&2
    stop_all
) 200>${LOCK_FILE}
