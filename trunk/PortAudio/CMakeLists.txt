# -*- mode: cmake -*-
# $Id$

# Check if cmake has the required version
CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

# Set name of our project.
PROJECT(PortAudio)

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

# Setup portaudio
FIND_PACKAGE(portaudio REQUIRED)
LIST(APPEND _libs ${portaudio_LIBRARY})

IF (APPLE)
   FIND_LIBRARY(COREAUDIO_LIBRARY CoreAudio)
   FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation )
ENDIF (APPLE)

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  ${portaudio_LIBRARY_INCLUDE_DIR}
  )

# add targets
FILE(GLOB files RELATIVE ${CMAKE_HOME_DIRECTORY} "*_main.cpp")
FOREACH(file ${files})
  STRING(REGEX REPLACE "_main.cpp$" "" target ${file})
  MESSAGE(STATUS "adding target: " ${target})
  ADD_EXECUTABLE(${target} ${file})
  TARGET_LINK_LIBRARIES(${target} ${_libs})
ENDFOREACH(file ${files})
