#!/bin/bash
# $Id

svn co http://libperseus-sdr.googlecode.com/svn/tags/REL-0.2 libperseus-sdr
cd libperseus-sdr
patch < ../libperseus-sdr-REL-0.2.diff