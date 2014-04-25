#!/bin/bash
# $Id$

# starts fltk_spec


source functions.sh

function find_port {
    cat config/multi_downconvert_`where_am_I`.xml \
	| awk '/port=/{print substr($1,7,5); exit}'
}

port=`find_port`

case $1 in
    EFR)
	./bin/fltk_spec -p $port -s DC_132000_EFR -o 50 -d 0.4 --fMin_kHz 125 --fMax_kHz 140 --sMin_dB -57 --sMax_dB -45
	;;
    DCF77)
	./bin/fltk_spec -p $port -s DC_077500_DCF -o 75 -d 0.5 --fMin_kHz  77 --fMax_kHz 78 --sMin_dB -57 --sMax_dB -45
	;;
    MSF)
	./bin/fltk_spec -p $port -s DC_060000_MSF -o 75 -d 0.5 --fMin_kHz  58 --fMax_kHz 61 --sMin_dB -57 --sMax_dB -45
	;;
    NavyMSK)
	./bin/fltk_spec -p $port -s DC_020000_MSKs -o 50 -d 1 --fMin_kHz 15 --fMax_kHz 25 --sMin_dB -75 --sMax_dB -45
	;;
    *)
	echo Unknown signal specification. Choose one from: EFR,DCF77,MSF,NavyMSK
	;;
esac
