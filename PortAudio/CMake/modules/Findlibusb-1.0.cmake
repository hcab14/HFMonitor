# -*- mode: cmake -*-
# $Id$
#
# libusb-1.0_INCLUDE_DIR
# libusb-1.0_LIBRARIES
# libusb-1.0_FOUND = true if libusb-1.0 is found

FIND_PATH(libusb-1.0_INCLUDE_DIR libusb-1.0/libusb.h
  /usr/include
  /usr/local/include
  /opt/local/include
  )

SET(_libusb libusb-1.0)
IF (APPLE)
  SET(_libusb libusb-1.0.dylib)
ENDIF (APPLE)

FIND_LIBRARY(libusb-1.0_LIBRARY NAMES ${_libusb} PATH 
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  ) 

IF (libusb-1.0_INCLUDE_DIR AND libusb-1.0_LIBRARY)
  SET(libusb-1.0_FOUND TRUE)
ENDIF (libusb-1.0_INCLUDE_DIR AND libusb-1.0_LIBRARY)

IF (libusb-1.0_FOUND)
  IF (NOT libusb-1.0_FIND_QUIETLY)
    MESSAGE(STATUS "Found libusb-1.0: ${libusb-1.0_LIBRARY}")
  ENDIF (NOT libusb-1.0_FIND_QUIETLY)
ELSE (libusb-1.0_FOUND)
  IF (libusb-1.0_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find libusb-1.0")
  ENDIF (libusb-1.0_FIND_REQUIRED)
ENDIF (libusb-1.0_FOUND)