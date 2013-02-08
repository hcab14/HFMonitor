# -*- mode: cmake -*-
# $Id$

SET(_programs)

# add targets
FILE(GLOB files RELATIVE ${CMAKE_HOME_DIRECTORY}/test "*_main.cpp")
FOREACH(file ${files})
  STRING(REGEX REPLACE "_main.cpp$" "" target ${file})
  MESSAGE(STATUS "adding target: " ${target})
  ADD_EXECUTABLE(${target} ${file})
  TARGET_LINK_LIBRARIES(${target} ${_libs})
  LIST(APPEND _programs ${target})
ENDFOREACH(file ${files})

INSTALL(TARGETS ${_programs} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
