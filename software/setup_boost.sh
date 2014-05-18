#!/bin/bash
# $Id$

boost_root=modular-boost
user_config=$boost_root/tools/build/src/user-config.jam

cp $boost_root/tools/build/example/user-config.jam $user_config

echo "setup_boost.sh " `whoami`

if [ `whoami` == chm ]; then
    cat <<EOF >> $user_config
using python : 2.6 : /usr/bin/python26 : /usr/include/python2.6 : /usr/lib ;

EOF
    
    echo "CC=$CC"
    if [ X$CC == X/usr/bin/gcc44 ]; then
	echo "using gcc44"
	cat <<EOF >> $user_config
using gcc : : /usr/bin/g++44 ; 

EOF
	
    fi
fi

