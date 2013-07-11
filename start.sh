#!/bin/bash
# $Id$

source functions.sh

(   flock -n -e 200 || { echo "This script $0 is currently locked"; exit 1; } >&2
    start_all
) 200>${LOCK_FILE}
