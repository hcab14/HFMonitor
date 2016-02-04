# -*- mode: Makefile -*-
# $Id$

ALL: boost/bjam fltk/bin/fltk-config sqlite3/bin/sqlite3

boost/bjam: boost_1_60_0/b2
	cd boost_1_60_0 && ./b2 -j4 && cd .. && rm -f boost && ln -s modular-boost boost

boost_1_60_0/b2: boost_1_60_0/bootstrap.sh
	cd boost_1_60_0 && ./bootstrap.sh && cd ..

boost_1_60_0/bootstrap.sh: boost_1_60_0.tar.bz2 
	bzip2 -dc boost_1_60_0.tar.bz2 | tar xvf -

modular-boost.tar.bz2:
	wget http://downloads.sourceforge.net/project/boost/boost/1.60.0/boost_1_60_0.tar.bz2


fltk/bin/fltk-config: fltk-1.3.2/insted/bin/fltk-config
	rm -f fltk && ln -s fltk-1.3.2 fltk

fltk-1.3.2/insted/bin/fltk-config: fltk-1.3.2/Makefile
	cd fltk-1.3.2 && make -j4 install && cd ..

fltk-1.3.2/Makefile: | fltk-1.3.2-source.tar.gz
	gzip -dc fltk-1.3.2-source.tar.gz  | tar xvf - && cd fltk-1.3.2 && mkdir insted &&./configure --prefix=`pwd`/insted && cd ..

fltk-1.3.2-source.tar.gz:
	wget http://fltk.org/pub/fltk/1.3.2/fltk-1.3.2-source.tar.gz

sqlite3/bin/sqlite3: sqlite-autoconf-3080403/configure
	cd sqlite-autoconf-3080403 && ./configure --prefix=`pwd`/../sqlite3 && make install && cd ../

sqlite-autoconf-3080403/configure: | sqlite-autoconf-3080403.tar.gz
	gzip -dc sqlite-autoconf-3080403.tar.gz | tar xvf -

sqlite-autoconf-3080403.tar.gz:
	wget https://sqlite.org/2014/sqlite-autoconf-3080403.tar.gz

CLEAN:
	rm fltk boost
