# -*- mode: cmake -*-
# $Id$


#
# USE_OPENCL
# USE_CUDA
#

# Check if cmake has the required version
CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

# Set name of our project.
PROJECT(HFMonitor)

SET(CPACK_GENERATOR "RPM")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Christoph Mayer") #required
INCLUDE(CPack)

EXEC_PROGRAM("${CMAKE_HOME_DIRECTORY}/get_git_version_string.sh ${CMAKE_HOME_DIRECTORY}/" OUTPUT_VARIABLE SVN_VERSION_STRING)
MESSAGE("VERSION=${SVN_VERSION_STRING}")

#SET(SVN_VERSION_STRING \"${SVN_VERSION_STRING}\")
SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_CXX_FLAGS         "-Wall -DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG -DSVN_VERSION_STRING=\"${SVN_VERSION_STRING}\"")

SET(CMAKE_CXX_FLAGS "-Wall -DSVN_VERSION_STRING=\"\\\"${SVN_VERSION_STRING}\\\"\"")
IF(!APPLE)
	 SET(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -DNDEBUG -msse3 -mssse3 -mcx16 -msse3 -msse4.1 -mpopcnt   -mmmx  -msse -msse2 -mfpmath=sse")
ELSE(!APPLE)
	SET(CMAKE_CXX_FLAGS_RELEASE "-g -O2 -DNDEBUG -DBOOST_LOG_DYN_LINK -Wno-unused-local-typedefs")
ENDIF(!APPLE)
SET(CMAKE_CXX_FLAGS_DEBUG "-g")
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# where to look first for cmake modules
SET(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/CMake/modules;${CMAKE_HOME_DIRECTORY}/software/install/lib/cmake/volk")

SET(CMAKE_PREFIX_PATH "${CMAKE_HOME_DIRECTORY}/software/install/")

SET(CMAKE_INSTALL_PREFIX ${CMAKE_HOME_DIRECTORY}/bin)

# List of libraries
SET(_libs)

IF (NOT APPLE)
   LIST(APPEND _libs "-lrt")
ENDIF (NOT APPLE)

# Setup FFTW3
FIND_PACKAGE(FFTW3 REQUIRED COMPONENTS single double threads)
LIST(APPEND _libs ${FFTW3_LIBRARIES})

# fin python  libraries
FIND_PACKAGE(PythonLibs REQUIRED)
LIST(APPEND _libs ${PYTHON_LIBRARY})

MESSAGE("USE_OPENCL=${USE_OPENCL}")

IF (USE_OPENCL)
   EXECUTE_PROCESS(COMMAND "make" "USE_OPENCL=1" WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}/software)
ELSE (USE_OPENCL)
   EXECUTE_PROCESS(COMMAND "make" WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY}/software)
ENDIF (USE_OPENCL)

#SET(Boost_USE_STATIC_LIBS   ON)
SET(Boost_USE_MULTITHREADED ON)
SET(BOOST_ROOT ${CMAKE_HOME_DIRECTORY}/software/install)
SET(BOOSTROOT ${CMAKE_HOME_DIRECTORY}/software/install)
SET(Boost_NO_SYSTEM_PATHS ON)
FIND_PACKAGE(Boost 1.60 REQUIRED COMPONENTS atomic chrono context log log_setup filesystem iostreams date_time regex program_options system coroutine thread python)
LIST(APPEND _libs ${Boost_LIBRARIES})

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

#Setup OpenCL
IF (USE_OPENCL)
  MESSAGE("********************************************************************************")
  FIND_PACKAGE(OpenCL REQUIRED)
  LIST(APPEND _libs ${OPENCL_LIBRARIES})
  INCLUDE_DIRECTORIES(${OPENCL_INCLUDE_DIRS})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CUDA")
ENDIF (USE_OPENCL)

# Setup FLTK
SET(FLTK_SKIP_OPENGL true)
SET(FLTK_SKIP_FLUID true)
SET(FLTK_PATH "${CMAKE_HOME_DIRECTORY}/software/install/bin")
FIND_PACKAGE(FLTK REQUIRED)
SET(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS}         ${FLTK_CXXFLAGS}")
IF (APPLE)
  SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS}  ${FLTK_LDFLAGS}")
ELSE (APPE)
  LIST(APPEND _libs ${FLTK_LDFLAGS})
ENDIF (APPLE)


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
IF (USE_CUDA)
  SET(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-6.5/)
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
ENDIF (USE_CUDA)
#volk
SET(ENV{VOLK_DIR} "${CMAKE_HOME_DIRECTORY}/software/install/")
FIND_PACKAGE(Volk REQUIRED)
IF(VOLK_FOUND)
  LIST(APPEND _libs ${VOLK_LIBRARIES})
  INCLUDE_DIRECTORIES(${VOLK_INCLUDE_DIRS})
ENDIF(VOLK_FOUND)

# Setup clFFT
SET(CLFFT_ROOT "${CMAKE_HOME_DIRECTORY}/software/install/")
FIND_PACKAGE(clFFT)
IF(CLFFT_FOUND)
  LIST(APPEND _libs ${CLFFT_LIBRARIES})
  INCLUDE_DIRECTORIES(
    ${CLFFT_INCLUDE_DIRS}
    )
  IF(APPLE)
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework OpenCL")
#  ELSE(APPLE)
#    LIST(APPEND _libs OpenCL)
  ENDIF(APPLE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CLFFT")
ENDIF(CLFFT_FOUND)

LIST(APPEND _libs pthread)

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  ${libusb-1.0_INCLUDE_DIR}
  ${FFTW3_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIR}
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

