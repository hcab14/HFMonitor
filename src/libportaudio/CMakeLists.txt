# -*- mode: cmake -*-
# $Id$

ADD_LIBRARY(portaudio_cpp portaudio.cpp)
LIST(APPEND _libs portaudio_cpp)

SET(_libs ${_libs} PARENT_SCOPE)
