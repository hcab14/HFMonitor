# -*- mode: cmake; indent-tabs-mode: nil -*-
# $Id$
#
# portaudio_INCLUDE_DIR
# portaudio_LIBRARIES
# portaudio_FOUND = true if portaudio is found

FIND_PATH(portaudio_INCLUDE_DIR portaudio.h
  /usr/include
  /usr/local/include
  /opt/local/include
  )

FIND_LIBRARY(portaudio_LIBRARY NAMES portaudio PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  ) 

IF (portaudio_INCLUDE_DIR AND portaudio_LIBRARY)
  SET(portaudio_FOUND TRUE)
ENDIF (portaudio_INCLUDE_DIR AND portaudio_LIBRARY)

IF (portaudio_FOUND)
  IF (NOT portaudio_FIND_QUIETLY)
    MESSAGE(STATUS "Found portaudio: ${portaudio_LIBRARY}")
  ENDIF (NOT portaudio_FIND_QUIETLY)
ELSE (portaudio_FOUND)
  IF (portaudio_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find portaudio")
  ENDIF (portaudio_FIND_REQUIRED)
ENDIF (portaudio_FOUND)