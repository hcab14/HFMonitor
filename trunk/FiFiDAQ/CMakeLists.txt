# -*- mode: cmake; indent-tabs-mode: nil -*-
# $Id$
#
# Copyright 2010-2011 Christoph Mayer
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Check if cmake has the required version
CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

# Set name of our project.
PROJECT(USB)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
SET(CMAKE_CXX_FLAGS_DEBUG   "-g -Wall")
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# where to look first for cmake modules
SET(CMAKE_MODULE_PATH "./CMake/modules")

# List of libraries
SET(_libs)

# Setup BOOST
SET(BOOST_ROOT /opt/local)
SET(Boost_USE_STATIC_LIBS   ON)
SET(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE(Boost 1.40 REQUIRED COMPONENTS system thread filesystem date_time)
LIST(APPEND _libs ${Boost_LIBRARIES})

# Setup libusb-1.0
FIND_PACKAGE(libusb-1.0 REQUIRED)
LIST(APPEND _libs ${libusb-1.0_LIBRARY})

# Setup portaudio
FIND_PACKAGE(Portaudio REQUIRED)
LIST(APPEND _libs ${portaudio_LIBRARY})

# Setup FFTW3
IF (NOT APPLE)
  FIND_PACKAGE(FFTW3 REQUIRED COMPONENTS single double threads )
  MESSAGE(STATUS "fftw3 ${FFTW3_LIBRARIES}")
  LIST(APPEND _libs ${FFTW3_LIBRARIES})
ENDIF (NOT APPLE)

FIND_PACKAGE(octave REQUIRED)
LIST(APPEND _libs ${OCTAVE_LIBRARY_RELEASE})
LINK_DIRECTORIES(${OCTAVE_LINK_DIRS})

IF (APPLE)
  FIND_LIBRARY(COREAUDIO_LIBRARY CoreAudio)
  FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation)
  LIST(APPEND _libs "-lfftw3 -lfftw3f")
ENDIF (APPLE)

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  ${libusb-1.0_INCLUDE_DIR}
  ${portaudio_LIBRARY_INCLUDE_DIR}
  ${OCTAVE_INCLUDE_DIRS}
  ${FFTW3_INCLUDE_DIR}
  )

ADD_LIBRARY(portaudio_cpp portaudio.cpp)
LIST(APPEND _libs portaudio_cpp)

ADD_LIBRARY(fifi_control_cpp libusb1.cpp fifi_control.cpp)
LIST(APPEND _libs fifi_control_cpp)

# add targets
FILE(GLOB files RELATIVE ${CMAKE_HOME_DIRECTORY} "*_main.cpp")
FOREACH(file ${files})
  STRING(REGEX REPLACE "_main.cpp$" "" target ${file})
  MESSAGE(STATUS "adding target: " ${target})
  ADD_EXECUTABLE(${target} ${file})
  TARGET_LINK_LIBRARIES(${target} ${_libs})
ENDFOREACH(file ${files})
