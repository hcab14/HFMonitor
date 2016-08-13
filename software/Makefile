# -*- mode: Makefile -*-
# $Id$

ALL: install/share/clFFT/examples install/lib/libboost_log.so.1.60.0 install/bin/volk_modtool install/bin/fltk-config install/bin/sqlite3

install/share/clFFT/examples: clFFT
ifdef USE_OPENCL
	cd clFFT && mkdir -p build && cd build && cmake -DCMAKE_INSTALL_PREFIX:PATH=`pwd`/../../install ../src && make -j4 install
endif
clFFT:
	git clone -b v2.10.0 https://github.com/clMathLibraries/clFFT.git

install/bin/volk_modtool: volk install/lib/libboost_log.so
	cd volk && mkdir -p build && cd build && cmake -DBOOST_ROOT=`pwd`/../../install -DPYTHON_EXECUTABLE=`which python2.6` -DCMAKE_INSTALL_PREFIX:PATH=`pwd`/../../install ../ && make -j4 install
volk:
	git clone -b v1.3 https://github.com/gnuradio/volk.git volk

install/lib/libboost_log.so.1.60.0: boost_1_60_0
	cd boost_1_60_0 && ./bootstrap.sh --prefix=`pwd`/../install --without-libraries=container && ./b2 -j4 install
boost_1_60_0:
	wget http://downloads.sourceforge.net/project/boost/boost/1.60.0/boost_1_60_0.tar.bz2 -O - | bzip2 -dc - | tar xvf -


install/bin/fltk-config: fltk-1.3.x-r11157
	cd fltk-1.3.x-r11157 && ./autogen.sh --prefix=`pwd`/../install && make -j4 install
fltk-1.3.x-r11157:
	wget http://fltk.org/pub/fltk/snapshots/fltk-1.3.x-r11157.tar.bz2 -O - | bzip2 -dc - | tar xvf -

install/bin/sqlite3: sqlite-autoconf-3080704
	cd sqlite-autoconf-3080704 && ./configure --prefix=`pwd`/../install && make install && cd ../
sqlite-autoconf-3080704:
	wget https://sqlite.org/2014/sqlite-autoconf-3080704.tar.gz -O - | gzip -dc - | tar xvf - 

CLEAN:
	rm -fr fltk* boost* sqlite*
