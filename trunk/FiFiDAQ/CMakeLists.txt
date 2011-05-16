# -*- mode: cmake -*-
# $Id$

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

IF (APPLE)
   FIND_LIBRARY(COREAUDIO_LIBRARY CoreAudio)
   FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation )
ENDIF (APPLE)

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  ${libusb-1.0_INCLUDE_DIR}
  ${portaudio_LIBRARY_INCLUDE_DIR}
  )


ADD_LIBRARY(portaudio_cpp portaudio.cpp)
LIST(APPEND _libs portaudio_cpp)

#ADD_LIBRARY(libusb1 libusb1.cpp)
#LIST(APPEND _libs libusb1)
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
