# -*- mode: cmake -*-
# $Id$

ADD_LIBRARY(perseus_control_cpp perseus_control.cpp)
ADD_LIBRARY(fx2_control_cpp fx2_control.cpp)
LIST(APPEND _libs perseus_control_cpp)
LIST(APPEND _libs fx2_control_cpp)
