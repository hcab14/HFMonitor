# -*- mode: cmake -*-
# $Id$

# Check if cmake has the required version
CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

# Set name of our project.
PROJECT(HFMonitor)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS         "-Wall -DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
SET(CMAKE_CXX_FLAGS_DEBUG   "-g")
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# where to look first for cmake modules
SET(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/CMake/modules")

# List of libraries
SET(_libs)

# Setup FFTW3
FIND_PACKAGE(FFTW3 REQUIRED COMPONENTS single double threads)
LIST(APPEND _libs ${FFTW3_LIBRARIES})

# Setup BOOST
SET(BOOST_ROOT /opt/local)
SET(Boost_USE_STATIC_LIBS   ON)
SET(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE(Boost 1.40 REQUIRED COMPONENTS system thread filesystem date_time regex program_options)
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

# Setup FLTK
#FIND_PACKAGE(FLTK REQUIRED)
#LIST(APPEND CMAKE_CXXFLAGS ${FLTK_CXXFLAGS})
#LIST(APPEND CMAKE_EXE_LINKER_FLAGS ${FLTK_LDFLAGS})

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  ${PerseusSDR_INCLUDE_DIR}
  ${FFTW3_INCLUDE_DIR}
  "./include"
  )

ADD_SUBDIRECTORY(src)

ADD_EXECUTABLE(test_perseus test_perseus_main.cpp)
TARGET_LINK_LIBRARIES(test_perseus ${_libs} usb1_cpp perseus_control_cpp fx2_control_cpp)

#ADD_EXECUTABLE(test_fltk_main test_fltk_main.cpp)
#TARGET_LINK_LIBRARIES(test_fltk_main ${_libs})

ADD_EXECUTABLE(test_goertzel_main test_goertzel_main.cpp)
TARGET_LINK_LIBRARIES(test_goertzel_main ${_libs})

ADD_EXECUTABLE(test_client_main test_client_main.cpp)
TARGET_LINK_LIBRARIES(test_client_main ${_libs})

ADD_EXECUTABLE(test_fir test_fir_main.cpp)
TARGET_LINK_LIBRARIES(test_fir ${_libs})

ADD_EXECUTABLE(test_regex test_regex_main.cpp)
TARGET_LINK_LIBRARIES(test_regex ${_libs})

ADD_EXECUTABLE(test_client_FFTtoFile test_client_FFTtoFile_main.cpp)
TARGET_LINK_LIBRARIES(test_client_FFTtoFile ${_libs} FFTAction_cpp processor_cpp)

ADD_EXECUTABLE(test_client_FFTtoBC test_client_FFTtoBC_main.cpp)
TARGET_LINK_LIBRARIES(test_client_FFTtoBC ${_libs} FFTAction_cpp processor_cpp)

ADD_EXECUTABLE(test_multi_downconvert test_multi_downconvert_main.cpp)
TARGET_LINK_LIBRARIES(test_multi_downconvert ${_libs} FFTAction_cpp processor_cpp)

ADD_EXECUTABLE(test_broadcaster test_broadcaster_main.cpp)
TARGET_LINK_LIBRARIES(test_broadcaster ${_libs})

ADD_EXECUTABLE(test_multi_client test_multi_client_main.cpp)
TARGET_LINK_LIBRARIES(test_multi_client ${_libs} FFTAction_cpp processor_cpp)

ADD_EXECUTABLE(server_ls server_ls_main.cpp)
TARGET_LINK_LIBRARIES(server_ls ${_libs} processor_cpp)


