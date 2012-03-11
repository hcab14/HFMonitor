# -*- mode: cmake -*-
# $Id$

ADD_SUBDIRECTORY(libportaudio)
ADD_SUBDIRECTORY(libusb1.0)
ADD_SUBDIRECTORY(sdr)
ADD_SUBDIRECTORY(FFTProcessor)

# add targets
#FILE(GLOB files RELATIVE ${CMAKE_HOME_DIRECTORY} "*_main.cpp")
#FOREACH(file ${files})
#  STRING(REGEX REPLACE "_main.cpp$" "" target ${file})
#  MESSAGE(STATUS "adding target: " ${target})
#  ADD_EXECUTABLE(${target} ${file})
#  TARGET_LINK_LIBRARIES(${target} ${_libs})
#ENDFOREACH(file ${files})
