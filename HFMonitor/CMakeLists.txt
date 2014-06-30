# -*- mode: cmake -*-
# $Id$

# Check if cmake has the required version
CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

# Set name of our project.
PROJECT(HFMonitor)

SET(CPACK_GENERATOR "RPM")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Christoph Mayer") #required
INCLUDE(CPack)

EXEC_PROGRAM("${CMAKE_HOME_DIRECTORY}/get_svn_version_string.sh ${CMAKE_HOME_DIRECTORY}" OUTPUT_VARIABLE SVN_VERSION_STRING)
MESSAGE("VERSION=${SVN_VERSION_STRING}")

#SET(SVN_VERSION_STRING \"${SVN_VERSION_STRING}\")
SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_CXX_FLAGS         "-Wall -DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG -DSVN_VERSION_STRING=\"${SVN_VERSION_STRING}\"")

# -malign-double in on 32bit systems incompatible with BOOST
SET(CMAKE_CXX_FLAGS         "-Wall -DSVN_VERSION_STRING=\"${SVN_VERSION_STRING}\"")
SET(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -DNDEBUG -msse3 -mssse3 -mcx16 -msse3 -msse4.1 -mpopcnt   -mmmx  -msse -msse2 -mfpmath=sse")
SET(CMAKE_CXX_FLAGS_DEBUG   "-g")
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# where to look first for cmake modules
SET(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/CMake/modules")

SET(CMAKE_INSTALL_PREFIX ${CMAKE_HOME_DIRECTORY}/bin)

# List of libraries
SET(_libs)

# Setup FFTW3
FIND_PACKAGE(FFTW3 REQUIRED COMPONENTS single double threads)
LIST(APPEND _libs ${FFTW3_LIBRARIES})

# Setup BOOST
EXECUTE_PROCESS(COMMAND "make" WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}/software)
SET(Boost_NO_SYSTEM_PATHS ON)
SET(BOOST_ROOT ${CMAKE_HOME_DIRECTORY}/software/boost)
SET(BOOST_LIBRARYDIR ${CMAKE_HOME_DIRECTORY}/software/boost/stage/lib)
SET(Boost_USE_STATIC_LIBS   ON)
SET(Boost_USE_MULTITHREADED ON)
#FIND_PACKAGE(Boost 1.55 REQUIRED COMPONENTS log_setup log filesystem date_time regex program_options system  coroutine thread context )
FIND_PACKAGE(Boost 1.54 REQUIRED COMPONENTS log_setup log filesystem date_time regex program_options system  thread )
LIST(APPEND _libs ${Boost_LIBRARIES})
IF(!APPLE)
  LIST(APPEND _libs rt)
  LIST(APPEND _libs pthread)
ENDIF(!APPLE)

MESSAGE("Boost_LIBRARIES = ${Boost_LIBRARIES}")
MESSAGE("Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")

# Setup libperseus-sdr
#FIND_PACKAGE(PerseusSDR REQUIRED)
#LIST(APPEND _libs ${PerseusSDR_LIBRARY})

# Setup libusb-1.0
FIND_PACKAGE(libusb-1.0 REQUIRED)
LIST(APPEND _libs ${libusb-1.0_LIBRARY})

# Setup portaudio
FIND_PACKAGE(Portaudio REQUIRED)
LIST(APPEND _libs ${portaudio_LIBRARY})

# Setup FLTK
IF (NOT APPLE)
  SET(FLTK_PATH ${CMAKE_HOME_DIRECTORY}/software/fltk/insted/bin)
ELSE (NOT APPLE)
  SET(FLTK_PATH /opt/local/bin)
ENDIF(NOT APPLE)
FIND_PACKAGE(FLTK REQUIRED)
SET(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS}         ${FLTK_CXXFLAGS}")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}  ${FLTK_LDFLAGS}")
LIST(APPEND _libs ${FLTK_LDFLAGS})

FIND_PACKAGE(BZip2 REQUIRED)
#  BZIP2_NEED_PREFIX - this is set if the functions are prefixed with BZ2_
IF(BZIP2_FOUND)
  LIST(APPEND _libs ${BZIP2_LIBRARIES})
  INCLUDE_DIRECTORIES(${BZIP2_INCLUDE_DIR})
ENDIF(BZIP2_FOUND)

FIND_PACKAGE(Sqlite3 REQUIRED)
IF(SQLITE3_FOUND)
  LIST(APPEND _libs ${SQLITE3_LIBRARIES})
  INCLUDE_DIRECTORIES(${SQLITE3_INCLUDE_DIR})  
ENDIF(SQLITE3_FOUND)

# Setup CUDA (optional)
SET(CUDA_TOOLKIT_ROOT_DIR /opt/local/cuda-5.0/)
FIND_PACKAGE(CUDA)
IF(CUDA_FOUND)
  LIST(APPEND _libs ${CUDA_LIBRARIES})
  LIST(APPEND _libs ${CUDA_CUFFT_LIBRARIES})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CUDA")
  INCLUDE_DIRECTORIES(
    ${CUDA_INCLUDE_DIRS}
    ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
  )
ENDIF(CUDA_FOUND)

LIST(APPEND _libs "-lrt")

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  ${libusb-1.0_INCLUDE_DIR}
  ${FFTW3_INCLUDE_DIR}
  "./include"
  )

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)

Option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" ON)
If(BUILD_DOCUMENTATION)
  FIND_PACKAGE(Doxygen)
  IF (NOT DOXYGEN_FOUND)
    MESSAGE(FATAL_ERROR "Doxygen is needed to build the documentation. Please install it correctly")
  ENDIF()
  #-- Configure the Template Doxyfile for our specific project
  Configure_file(Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
  #-- Add a custom target to run Doxygen when ever the project is built
  add_custom_target (Docs #ALL 
    COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
    SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
  # IF you do NOT want the documentation to be generated EVERY time you build the project
  # then leave out the 'ALL' keyword from the above command.
endif()
#SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "VLF Spectrum Monitoring")
#SET(CPACK_PACKAGE_VENDOR "Christoph Mayer")
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
#SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
#SET(CPACK_PACKAGE_VERSION_MAJOR "1")
#SET(CPACK_PACKAGE_VERSION_MINOR "3")
#SET(CPACK_PACKAGE_VERSION_PATCH "2")
#SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
#SET(CPACK_STRIP_FILES "bin/client_FFTtoBC")
#SET(CPACK_SOURCE_STRIP_FILES "")
#SET(CPACK_PACKAGE_EXECUTABLES 
#  "client_FFTtoBC" "Executable Description"
#  "client_FFTtoFile" "Executable Description"
#  "client_multi_downconvert" "Executable Description"
#  "multi_client perseus_server" "Executable Description"
#  "server_ls" "Executable Description"
#  )
