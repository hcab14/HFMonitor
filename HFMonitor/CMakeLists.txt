# -*- mode: cmake -*-
# $Id$

# Check if cmake has the required version
CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

# Set name of our project.
PROJECT(HFMonitor)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
SET(CMAKE_CXX_FLAGS_DEBUG   "-g -Wall")
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# where to look first for cmake modules
SET(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/CMake/modules")

# List of libraries
SET(_libs)

# Setup FFTW3
FIND_PACKAGE(FFTW3 REQUIRED COMPONENTS single double threads )
LIST(APPEND _libs ${FFTW3_LIBRARIES})

# Setup BOOST
SET(BOOST_ROOT /opt/local)
SET(Boost_USE_STATIC_LIBS   ON)
SET(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE(Boost 1.40 REQUIRED COMPONENTS system thread filesystem date_time)
LIST(APPEND _libs ${Boost_LIBRARIES})

# Setup libperseus-sdr
FIND_PACKAGE(PerseusSDR REQUIRED)
LIST(APPEND _libs ${PerseusSDR_LIBRARY})

# Setup libusb-1.0
FIND_PACKAGE(libusb-1.0 REQUIRED)
LIST(APPEND _libs ${libusb-1.0_LIBRARY})

# Setup portaudio
FIND_PACKAGE(Portaudio REQUIRED)
LIST(APPEND _libs ${portaudio_LIBRARY})

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  ${PerseusSDR_INCLUDE_DIR}
  ${FFTW3_INCLUDE_DIR}
  "./include"
  )

ADD_SUBDIRECTORY(src)

ADD_EXECUTABLE(test_perseus test_perseus_main.cpp)
TARGET_LINK_LIBRARIES(test_perseus ${_libs} usb1_cpp perseus_control_cpp)