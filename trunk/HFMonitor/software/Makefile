# -*- mode: Makefile -*-
# $Id$

ALL: install/share/clFFT/examples install/bin/volk_profile #boost_1_57_0/bjam fltk/insted/bin/fltk-config sqlite3/bin/sqlite3

install/share/clFFT/examples: clFFT
	cd clFFT && mkdir -p build && cd build && cmake -DCMAKE_INSTALL_PREFIX:PATH=../../install ../src && make -j4 install
clFFT:
	git clone -b v2.10.0 https://github.com/clMathLibraries/clFFT.git

install/bin/volk_profile: volk
	cd volk && mkdir -p build && cd build && cmake -DCMAKE_INSTALL_PREFIX:PATH=../../install ../ && make -j4 install
volk:
	git clone -b v1.2.1 https://github.com/gnuradio/volk.git volk

boost/bjam: boost_1_60_0/b2
	cd boost_1_60_0 && ./b2 -j4 && cd .. && rm -f boost && ln -s modular-boost boost
boost_1_60_0/b2: boost_1_60_0/bootstrap.sh
	cd boost_1_60_0 && ./bootstrap.sh && cd ..
boost_1_60_0/bootstrap.sh: boost_1_60_0.tar.bz2 
	bzip2 -dc boost_1_60_0.tar.bz2 | tar xvf -
boost_1_60_0.tar.bz2:
	wget http://downloads.sourceforge.net/project/boost/boost/1.60.0/boost_1_60_0.tar.bz2


fltk/insted/bin/fltk-config: fltk-1.3.3/insted/bin/fltk-config
	rm -f fltk && ln -s fltk-1.3.3 fltk
fltk-1.3.3/insted/bin/fltk-config: fltk-1.3.3/Makefile
	cd fltk-1.3.3 && make -j4 install && cd ..
fltk-1.3.3/Makefile: fltk-1.3.3-source.tar.gz
	gzip -dc fltk-1.3.3-source.tar.gz | tar xvf - && cd fltk-1.3.3 && mkdir -p insted &&./configure --prefix=`pwd`/insted && cd .. && touch fltk-1.3.3/Makefile
fltk-1.3.3-source.tar.gz:
	wget http://fltk.org/pub/fltk/1.3.3/fltk-1.3.3-source.tar.gz && touch fltk-1.3.3-source.tar.gz


sqlite3/bin/sqlite3: sqlite-autoconf-3080704/configure
	cd sqlite-autoconf-3080704 && ./configure --prefix=`pwd`/../sqlite3 && make install && cd ../
sqlite-autoconf-3080704/configure: | sqlite-autoconf-3080704.tar.gz
	gzip -dc sqlite-autoconf-3080704.tar.gz | tar xvf -
sqlite-autoconf-3080704.tar.gz:
	wget https://sqlite.org/2014/sqlite-autoconf-3080704.tar.gz

CLEAN:
	rm -fr fltk* boost* sqlite*
