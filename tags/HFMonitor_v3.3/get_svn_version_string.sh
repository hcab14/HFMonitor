#!/bin/bash
# $Id$

# taken from http://stackoverflow.com/a/5027832
function a2r {
    local source=$1
    local target=$2

    local common_part=$source
    local back=
    while [ "${target#$common_part}" = "${target}" ]; do
	common_part=$(dirname $common_part)
	back="../${back}"
    done
    
    echo ${back}${target#$common_part/}
}

function get_version_string {
    local rel_path=$1
    local path=""
    [ XX`uname` == XXDarwin ] && path=/opt/local/bin/
    echo path=$path
    ${path}svnversion ${rel_path}
    ${path}svn info ${rel_path} | awk '/^Last Changed Author:/ {print $4}'
    ${path}svn info ${rel_path} | awk '/^Last Changed Date:/ {print $4, $5, $6}'
}

rel_path=`a2r $PWD/ $1/`
version_string="`get_version_string $rel_path`"
echo $version_string

