# -*- mode: cmake -*-
# $Id$

# add FFTAction factory library
ADD_LIBRARY(FFTAction_cpp FFTAction.cpp)
LIST(APPEND _libs FFTAction_cpp)

SET(_libs ${_libs} PARENT_SCOPE)
