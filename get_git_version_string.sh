#!/bin/bash
# $Id$

cd $1
git show -s --pretty=format:"%H %aD %cn"
echo

