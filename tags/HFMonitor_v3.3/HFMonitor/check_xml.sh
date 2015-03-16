#!/bin/bash
# $Id$

for i in config/*.xml; do
    xmllint --valid $i > /dev/null 
    s=$?
    echo -n "xmllint "
    [ $s != 0 ] && echo FAILED: $i || echo OK:     $i
done