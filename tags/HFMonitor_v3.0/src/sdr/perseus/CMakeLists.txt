# -*- mode: cmake -*-
# $Id$

ADD_LIBRARY(perseus_control_cpp perseus_control.cpp)
LIST(APPEND _libs perseus_control_cpp)

ADD_LIBRARY(fx2_control_cpp fx2_control.cpp)
LIST(APPEND _libs fx2_control_cpp)

SET(_libs ${_libs} PARENT_SCOPE)