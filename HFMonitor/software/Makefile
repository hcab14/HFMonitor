# -*- mode: Makefile -*-
# $Id$

ALL: boost_1_57_0/bjam fltk/bin/fltk-config sqlite3/bin/sqlite3

boost_1_57_0/bjam: boost_1_57_0/b2
	cd boost_1_57_0 && ./b2 toolset=darwin-4.8 && cd .. && rm -f boost && ln -s boost_1_57_0 boost

boost_1_57_0/b2: boost_1_57_0 ./setup_boost.sh
	./setup_boost.sh && cd boost_1_57_0 && ./bootstrap.sh && cd ..

boost_1_57_0/bootstrap.sh: boost_1_57_0
	bzip2 -dc boost_1_57_0.tar.bz2 | tar --exclude "*/doc/*" -xvf -

#boost_1_57_0.tar.bz2:
#	wget http://nz23-18.ifj.edu.pl/boost/boost_1_57_0.tar.bz2


fltk/bin/fltk-config: fltk-1.3.2/insted/bin/fltk-config
	rm -f fltk && ln -s fltk-1.3.2 fltk

fltk-1.3.2/insted/bin/fltk-config: fltk-1.3.2/Makefile
	cd fltk-1.3.2 && make -j4 install && cd ..

fltk-1.3.2/Makefile: | fltk-1.3.2-source.tar.gz
	gzip -dc fltk-1.3.2-source.tar.gz  | tar xvf - && cd fltk-1.3.2 && mkdir insted &&./configure --prefix=`pwd`/insted && cd ..

fltk-1.3.2-source.tar.gz:
	wget http://fltk.org/pub/fltk/1.3.2/fltk-1.3.2-source.tar.gz

sqlite3/bin/sqlite3: sqlite-autoconf-3080704/configure
	cd sqlite-autoconf-3080704 && ./configure --prefix=`pwd`/../sqlite3 && make install && cd ../

sqlite-autoconf-3080704/configure: | sqlite-autoconf-3080704.tar.gz
	gzip -dc sqlite-autoconf-3080704.tar.gz | tar xvf -

sqlite-autoconf-3080704.tar.gz:
	wget https://sqlite.org/2014/sqlite-autoconf-3080704.tar.gz

CLEAN:
	rm fltk boost
