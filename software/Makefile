# -*- mode: Makefile -*-
# $Id$

INSTALL_PATH = $(shell echo `pwd`/install)

targets = install/lib/libboost_log.a install/bin/volk_modtool install/bin/fltk-config

ifndef APPLE
targets += install/bin/sqlite3
endif

ifdef USE_OPENCL
targets += install/share/clFFT/examples
endif

ALL: ${targets}

install/share/clFFT/examples: clFFT install/lib/libboost_log.a
	cd clFFT && mkdir -p build && cd build && cmake3 -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_PATH} -DBOOST_ROOT=${INSTALL_PATH} ../src && make -j4 install

clFFT:
	git clone https://github.com/clMathLibraries/clFFT.git

install/bin/volk_modtool: volk install/lib/libboost_log.a
	cd volk && mkdir -p build && cd build && cmake3 -DCMAKE_C_FLAGS="--std=gnu99" -DBOOST_ROOT=${INSTALL_PATH} -DPYTHON_EXECUTABLE=`which python2` -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_PATH} ../ && make -j4 install
volk:
	git clone https://github.com/gnuradio/volk.git volk

install/lib/libboost_log.a: boost_1_63_0
	cd boost_1_63_0 && ./bootstrap.sh --prefix=${INSTALL_PATH} --with-libraries=python,program_options,thread,coroutine,iostreams,locale,log,test && ./b2 -j4 --layout=system install
boost_1_63_0:
	wget http://downloads.sourceforge.net/project/boost/boost/1.63.0/boost_1_63_0.tar.bz2 -O - | bzip2 -dc - | tar xvf -


install/bin/fltk-config: fltk-1.3.x-r11157
	cd fltk-1.3.x-r11157 && ./autogen.sh --prefix=${INSTALL_PATH} && make -j4 install
fltk-1.3.x-r11157:
	wget http://fltk.org/pub/fltk/snapshots/fltk-1.3.x-r11157.tar.bz2 -O - | bzip2 -dc - | tar xvf -

install/bin/sqlite3: sqlite-autoconf-3080704
	cd sqlite-autoconf-3080704 && ./configure --prefix=${INSTALL_PATH} && make install && cd ../
sqlite-autoconf-3080704:
	wget https://sqlite.org/2014/sqlite-autoconf-3080704.tar.gz -O - | gzip -dc - | tar xvf -

clean:
	rm -fr fltk* boost* sqlite* install/* volk*
