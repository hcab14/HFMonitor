# -*- mode: cmake; indent-tabs-mode: nil -*-
# $Id$
#
# Copyright 2010-2011 Christoph Mayer
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# libusb-1.0_INCLUDE_DIR
# libusb-1.0_LIBRARIES
# libusb-1.0_FOUND = true if libusb-1.0 is found

FIND_PATH(libusb-1.0_INCLUDE_DIR libusb-1.0/libusb.h
  /usr/include
  /usr/local/include
  /opt/local/include
  )

FIND_LIBRARY(libusb-1.0_LIBRARY NAMES usb-1.0 PATHS
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