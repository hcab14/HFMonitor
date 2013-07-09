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

  execute_process(
    COMMAND ${FLTK_CONFIG_EXECUTABLE} --ldstaticflags
    OUTPUT_VARIABLE _fltk_config_ldflags
    RESULT_VARIABLE _fltk_config_failed)
  string(REGEX REPLACE "[\r\n]" " " FLTK_LDFLAGS "${_fltk_config_ldflags}")

endif(FLTK_CONFIG_EXECUTABLE)

MESSAGE("FLTK found: " ${FLTK_CXXFLAGS} ${FLTK_LDFLAGS})

MARK_AS_ADVANCED(
    FLTK_FOUND
    FLTK_CXXFLAGS
    FLTK_LDFLAGS
)