# -*- mode: cmake -*-
# $Id$

ADD_LIBRARY(usb1_cpp libusb1.cpp)
LIST(APPEND _libs usb1_cpp)

SET(_libs ${_libs} PARENT_SCOPE)