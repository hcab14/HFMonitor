#!/bin/bash
# $Id$

if [ `whoami`_`uname` == chm_Darwin ]; then
    echo toolset=darwin-4.8
else
    [ X$CC == X ] && CC=gcc
    cc_version=`$CC --version | awk '/gcc/ {print $3}'`
    echo toolset=gcc-$cc_version
fi
