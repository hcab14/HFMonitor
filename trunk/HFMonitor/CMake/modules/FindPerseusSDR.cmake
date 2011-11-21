# -*- mode: cmake -*-
# $Id$
#
# PerseusSDR_INCLUDE_DIR = perseus-sdr.h
# PerseusSDR_LIBRARY     = libperseus-sdr.a
# PerseusSDR_FOUND       = true if PerseusSDR is found

FIND_PATH(PerseusSDR_INCLUDE_DIR perseus-sdr.h 
  /usr/include
  /usr/local/include
  /opt/local/include
  )

FIND_LIBRARY(PerseusSDR_LIBRARY NAMES perseus-sdr PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  ) 

IF (PerseusSDR_INCLUDE_DIR AND PerseusSDR_LIBRARY)
  SET(PerseusSDR_FOUND TRUE)
ENDIF (PerseusSDR_INCLUDE_DIR AND PerseusSDR_LIBRARY)

IF (PerseusSDR_FOUND)
  IF (NOT PerseusSDR_FIND_QUIETLY)
    MESSAGE(STATUS "Found PerseusSDR: ${PerseusSDR_LIBRARY}")
  ENDIF (NOT PerseusSDR_FIND_QUIETLY)
ELSE (PerseusSDR_FOUND)
  IF (PerseusSDR_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find PerseusSDR")
  ENDIF (PerseusSDR_FIND_REQUIRED)
ENDIF (PerseusSDR_FOUND)