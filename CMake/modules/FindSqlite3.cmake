# -*- mode: cmake; indent-tabs-mode: nil -*-
# $Id$
#
# Copyright 2010-2012 Christoph Mayer
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
# - find Sqlite 3
# SQLITE3_INCLUDE_DIR - Where to find Sqlite 3 header files (directory)
# SQLITE3_LIBRARIES - Sqlite 3 library
# SQLITE3_FOUND - Set to TRUE if we found everything (library, includes and executable)

FIND_PATH  (SQLITE3_INCLUDE_DIR NAME sqlite3.h PATHS ${CMAKE_HOME_DIRECTORY}/software/install/sqlite3/include)
IF(NOT SQLITE3_INCLUDE_DIR)
  FIND_PATH(SQLITE3_INCLUDE_DIR NAME sqlite3.h PATHS /opt/local/include)
ENDIF(NOT SQLITE3_INCLUDE_DIR)

FIND_LIBRARY  (SQLITE3_LIBRARIES NAME sqlite3 PATHS ${CMAKE_HOME_DIRECTORY}/software/install/sqlite3/lib)
IF(NOT SQLITE3_LIBRARIES)
  FIND_LIBRARY(SQLITE3_LIBRARIES NAME sqlite3 PATHS /opt/local/lib)
ENDIF(NOT SQLITE3_LIBRARIES)

IF(SQLITE3_LIBRARIES AND SQLITE3_INCLUDE_DIR)
  SET(SQLITE3_FOUND TRUE)
ENDIF(SQLITE3_LIBRARIES AND SQLITE3_INCLUDE_DIR)

IF(SQLITE3_FOUND)
  IF(NOT SQLITE3_FIND_QUIETLY)
    MESSAGE(STATUS "Found Sqlite3 header file in ${SQLITE3_INCLUDE_DIR}")
    MESSAGE(STATUS "Found Sqlite3 libraries: ${SQLITE3_LIBRARIES}")
  ENDIF(NOT SQLITE3_FIND_QUIETLY)
ELSE(SQLITE3_FOUND)
  IF(SQLITE3_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find Sqlite3")
  ELSE(SQLITE3_FIND_REQUIRED)
    MESSAGE(STATUS "Sqlite3 was not found")
  ENDIF(SQLITE3_FIND_REQUIRED)
ENDIF(SQLITE3_FOUND)

MARK_AS_ADVANCED(
    SQLITE3_FOUND
    SQLITE3_INCLUDE_DIR
    SQLITE3_LIBRARIES
)
