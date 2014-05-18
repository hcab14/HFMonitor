# -*- mode: cmake -*-
# $Id$

ADD_SUBDIRECTORY(libportaudio)
ADD_SUBDIRECTORY(libusb1.0)
ADD_SUBDIRECTORY(sdr)
ADD_SUBDIRECTORY(FFTProcessor)

ADD_LIBRARY(logging_cpp logging.cpp)
TARGET_LINK_LIBRARIES(logging_cpp ${Boost_LIBRARIES})
LIST(APPEND _libs logging_cpp)

ADD_LIBRARY(processor_cpp processor.cpp)
LIST(APPEND _libs processor_cpp)

SET(_programs)

# add targets
FILE(GLOB files RELATIVE ${CMAKE_HOME_DIRECTORY}/src "*_main.cpp")
FOREACH(file ${files})
  STRING(REGEX REPLACE "_main.cpp$" "" target ${file})
  MESSAGE(STATUS "adding target: " ${target})
  ADD_EXECUTABLE(${target} ${file})
  TARGET_LINK_LIBRARIES(${target} ${_libs})
  LIST(APPEND _programs ${target})
ENDFOREACH(file ${files})

SET(_libs ${_libs} PARENT_SCOPE)

INSTALL(TARGETS ${_programs} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})

