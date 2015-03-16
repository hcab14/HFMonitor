# -*- mode: cmake -*-
# $Id$

ADD_LIBRARY(fifi_control_cpp fifi_control.cpp)
LIST(APPEND _libs fifi_control_cpp)

SET(_libs ${_libs} PARENT_SCOPE)