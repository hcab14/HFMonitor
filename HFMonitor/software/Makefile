# -*- mode: Makefile -*-
# $Id$

ALL: boost/bjam fltk/bin/fltk-config sqlite3/bin/sqlite3

boost/bjam: modular-boost/b2
	cd modular-boost && ./b2 headers && ./b2 -j4 && cd .. && rm -f boost && ln -s modular-boost boost

modular-boost/b2: modular-boost/bootstrap.sh ./setup_boost.sh
	./setup_boost.sh && cd modular-boost && ./bootstrap.sh && cd ..

modular-boost/bootstrap.sh: | modular-boost.tar.bz2
	bzip2 -dc modular-boost.tar.bz2 | tar xvf -

modular-boost.tar.bz2:
	wget http://nz23-18.ifj.edu.pl/boost/modular-boost.tar.bz2


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
