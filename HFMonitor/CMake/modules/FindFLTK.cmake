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

# FLTK_FOUND 
# FLTK_CXXFLAGS
# FLTK_LDFLAGS
# FLTK_LIBS

# use fltk-config
set(FLTK_CONFIG_EXECUTABLE FLTK_CONFIG_EXECUTABLE-NOTFOUND)
find_program(FLTK_CONFIG_EXECUTABLE NAME fltk-config 
                                    PATHS ${FLTK_PATH}
                                    NO_DEFAULT_PATH)
mark_as_advanced(FLTK_CONFIG_EXECUTABLE)

if(FLTK_CONFIG_EXECUTABLE)
  set(FLTK_FOUND 1)

  execute_process(
    COMMAND ${FLTK_CONFIG_EXECUTABLE} --cxxflags
    OUTPUT_VARIABLE _fltk_config_cxxflags
    RESULT_VARIABLE _fltk_config_failed)
  string(REGEX REPLACE "[\r\n]" " " FLTK_CXXFLAGS "${_fltk_config_cxxflags}")
  string(REGEX REPLACE " $" "" FLTK_CXXFLAGS "${FLTK_CXXFLAGS}")

  execute_process(
    COMMAND ${FLTK_CONFIG_EXECUTABLE} --ldflags
    OUTPUT_VARIABLE _fltk_config_ldflags
    RESULT_VARIABLE _fltk_config_failed)
  string(REGEX REPLACE "[\r\n]" " " FLTK_LDFLAGS "${_fltk_config_ldflags}")
  string(REGEX REPLACE " $" "" FLTK_LDFLAGS "${FLTK_LDFLAGS}")

endif(FLTK_CONFIG_EXECUTABLE)

MARK_AS_ADVANCED(
    FLTK_FOUND
    FLTK_CXXFLAGS
    FLTK_LDFLAGS
)
