#!/bin/bash
# $Id$

boost_root=boost_1_57_0
user_config=$boost_root/tools/build/src/user-config.jam

cp $boost_root/tools/build/example/user-config.jam $user_config

echo "setup_boost.sh " `whoami`

if [ `whoami`_`uname` == chm_Darwin ]; then
    cat <<EOF >> $user_config
using python : 2.5 : /usr/bin/python2.5 : /usr/include/python2.5 : /usr/lib/python2.5 ;

EOF
fi

if [ `whoami`_`uname` == chm_Linux ]; then
    cat <<EOF >> $user_config
using python : 2.6 : /usr/bin/python26 : /usr/include/python2.6 : /usr/lib ;

EOF
fi

if [ `whoami`_`uname` == chm_Darwin ]; then
    echo "CC=$CC"
    if [ X$CC != X ]; then
        ccc=`which $CXX`
	echo "using $CC $ccc"
	cat <<EOF >> $user_config
using darwin : 4.8 : $ccc ; 

EOF
	
    fi
fi

if [ `whoami`_`uname` == chm_Linux ]; then
    echo "CC=$CC"
    if [ X$CC != X ]; then
        ccc=`which $CXX`
	cc_version=`$CC --version | awk '/gcc/ {print $3}'`
	echo "using $CC $ccc $cc_version"
	cat <<EOF >> $user_config
using gcc : $cc_version : $ccc ; 

EOF
	
    fi
fi

cat $user_config
