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