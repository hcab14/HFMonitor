# -*- mode: Makefile -*-
# $Id$


ALL: boost fltk

boost: boost_1_53_0/b2
	cd boost_1_53_0 && ./b2 -j4 && cd .. && rm -f boost && ln -s boost_1_53_0 boost

boost_1_53_0/b2: boost_1_53_0
	cd boost_1_53_0; ./bootstrap.sh; cd ..

boost_1_53_0: boost_1_53_0.tar.bz2
	bzip2 -dc boost_1_53_0.tar.bz2 | tar xvf -

boost_1_53_0.tar.bz2:
	wget http://sourceforge.net/projects/boost/files/boost/1.53.0/boost_1_53_0.tar.bz2/download


fltk: fltk-1.3.2/insted/bin/fltk-config
	rm -f fltk && ln -s fltk-1.3.2 fltk

fltk-1.3.2/insted/bin/fltk-config: fltk-1.3.2
	cd fltk-1.3.2 && make -j4 install && cd ..

fltk-1.3.2: fltk-1.3.2-source.tar.gz
	gzip -dc fltk-1.3.2-source.tar.gz  | tar xvf - && cd fltk-1.3.2 && mkdir insted &&./configure --prefix=`pwd`/insted && cd ..

fltk-1.3.2-source.tar.gz:
	wget http://fltk.org/pub/fltk/1.3.2/fltk-1.3.2-source.tar.gz

CLEAN:
	rm fltk boost
