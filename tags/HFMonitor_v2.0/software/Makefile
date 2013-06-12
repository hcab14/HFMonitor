# -*- mode: Makefile -*-
# $Id$

.PHONY: boost_build

ALL: boost_build
	rm -f boost; ln -s boost_1_53_0 boost

boost_build: boost_1_53_0/b2
	cd boost_1_53_0; ./b2 -j4; cd ..

boost_1_53_0/b2: boost_1_53_0
	cd boost_1_53_0; ./bootstrap.sh; cd ..

boost_1_53_0: boost_1_53_0.tar.bz2
	bzip2 -dc boost_1_53_0.tar.bz2 | tar xvf -

boost_1_53_0.tar.bz2:
	wget http://sourceforge.net/projects/boost/files/boost/1.53.0/boost_1_53_0.tar.bz2/download