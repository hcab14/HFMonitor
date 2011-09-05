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
# adapted from http://code.google.com/p/biosignal/ (hg/cmake/modules/FindOctave.cmake)
#
# Try to find the build flags to compile octave shared objects (oct and mex files)
# Once done this will define
#
# OCTAVE_FOUND - if Coin3d is found
# OCTAVE_CXXFLAGS - extra flags
# OCTAVE_INCLUDE_DIRS - include directories
# OCTAVE_LINK_DIRS - link directories
# OCTAVE_LIBRARY_RELEASE - the relase version
# OCTAVE_LIBRARY_DEBUG - the debug version
# OCTAVE_LIBRARY - a default library, with priority debug.

# use mkoctfile
set(MKOCTFILE_EXECUTABLE MKOCTFILE_EXECUTABLE-NOTFOUND)
find_program(MKOCTFILE_EXECUTABLE NAME mkoctfile PATHS)
mark_as_advanced(MKOCTFILE_EXECUTABLE)

# use octave_config
set(OCTAVE_CONFIG_EXECUTABLE OCTAVE_CONFIG_EXECUTABLE-NOTFOUND)
find_program(OCTAVE_CONFIG_EXECUTABLE NAME octave-config PATHS)
mark_as_advanced(OCTAVE_CONFIG_EXECUTABLE)


if(MKOCTFILE_EXECUTABLE)
  set(OCTAVE_FOUND 1)

  execute_process(
    COMMAND ${MKOCTFILE_EXECUTABLE} -p ALL_CXXFLAGS
    OUTPUT_VARIABLE _mkoctfile_cppflags
    RESULT_VARIABLE _mkoctfile_failed)
  string(REGEX REPLACE "[\r\n]" " " _mkoctfile_cppflags "${_mkoctfile_cppflags}")
  execute_process(
    COMMAND ${MKOCTFILE_EXECUTABLE} -p INCFLAGS
    OUTPUT_VARIABLE _mkoctfile_includedir
    RESULT_VARIABLE _mkoctfile_failed)
  string(REGEX REPLACE "[\r\n]" " " _mkoctfile_includedir "${_mkoctfile_includedir}")
  string(REGEX REPLACE "-I" " " _mkoctfile_includedir "${_mkoctfile_includedir}")
  execute_process(
    COMMAND ${MKOCTFILE_EXECUTABLE} -p ALL_LDFLAGS
    OUTPUT_VARIABLE _mkoctfile_ldflags
    RESULT_VARIABLE _mkoctfile_failed)
  string(REGEX REPLACE "[\r\n]" " " _mkoctfile_ldflags "${_mkoctfile_ldflags}")
  execute_process(
    COMMAND ${MKOCTFILE_EXECUTABLE} -p LFLAGS
    OUTPUT_VARIABLE _mkoctfile_lflags
    RESULT_VARIABLE _mkoctfile_failed)
  string(REGEX REPLACE "[\r\n]" " " _mkoctfile_lflags "${_mkoctfile_lflags}")
  execute_process(
    COMMAND ${MKOCTFILE_EXECUTABLE} -p LIBS
    OUTPUT_VARIABLE _mkoctfile_libs
    RESULT_VARIABLE _mkoctfile_failed)
  string(REGEX REPLACE "[\r\n]" " " _mkoctfile_libs "${_mkoctfile_libs}")
  execute_process(
    COMMAND ${MKOCTFILE_EXECUTABLE} -p OCTAVE_LIBS
    OUTPUT_VARIABLE _mkoctfile_octlibs
    RESULT_VARIABLE _mkoctfile_failed)
  string(REGEX REPLACE "[\r\n]" " " _mkoctfile_octlibs "${_mkoctfile_octlibs}")
  set(_mkoctfile_libs "${_mkoctfile_libs} ${_mkoctfile_octlibs}")

  string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _mkoctfile_libs "${_mkoctfile_libs}")
  string(REGEX REPLACE "(^| )-l" "" _mkoctfile_libs "${_mkoctfile_libs}")

  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _mkoctfile_ldirs "${_mkoctfile_lflags}")
  string(REGEX REPLACE "(^| )-L" "" _mkoctfile_ldirs "${_mkoctfile_ldirs}")

  string(REGEX REPLACE "(^| )-l([./+-_\\a-zA-Z]*)" " " _mkoctfile_ldflags "${_mkoctfile_ldflags}")
  string(REGEX REPLACE "(^| )-L([./+-_\\a-zA-Z]*)" " " _mkoctfile_ldflags "${_mkoctfile_ldflags}")

  separate_arguments(_mkoctfile_includedir)

  set( OCTAVE_CXXFLAGS "${_mkoctfile_cppflags}" )
  set( OCTAVE_LINK_FLAGS "${_mkoctfile_ldflags}" )
  set( OCTAVE_INCLUDE_DIRS "${_mkoctfile_includedir}")
  set( OCTAVE_LINK_DIRS ${_mkoctfile_ldirs})
  set( OCTAVE_LIBRARY ${_mkoctfile_libs})

  set( OCTAVE_LIBRARY_RELEASE "${OCTAVE_LIBRARY}")
  set( OCTAVE_LIBRARY_DEBUG "${OCTAVE_LIBRARY}")
endif(MKOCTFILE_EXECUTABLE)
if(OCTAVE_CONFIG_EXECUTABLE)
  execute_process(
    COMMAND ${OCTAVE_CONFIG_EXECUTABLE} -p CANONICAL_HOST_TYPE
    OUTPUT_VARIABLE _octave_config_host_type
    RESULT_VARIABLE _octave_config_failed)
  string(REGEX REPLACE "[\r\n]""" _octave_config_host_type "${_octave_config_host_type}")
  execute_process(
    COMMAND ${OCTAVE_CONFIG_EXECUTABLE} -p API_VERSION
    OUTPUT_VARIABLE _octave_config_api_version
    RESULT_VARIABLE _octave_config_failed)
  string(REGEX REPLACE "[\r\n]""" _octave_config_api_version "${_octave_config_api_version}")
  execute_process(
    COMMAND ${OCTAVE_CONFIG_EXECUTABLE} -p LOCALVEROCTFILEDIR
    OUTPUT_VARIABLE _octave_config_localveroctfiledir
    RESULT_VARIABLE _octave_config_failed)
  string(REGEX REPLACE "[\r\n]""" _octave_config_localveroctfiledir "${_octave_config_api_version}")

  set( OCTAVE_HOST_TYPE "${_octave_config_host_type}" )
  set( OCTAVE_API_VERSION "${_octave_config_api_version}" )
  set( OCTAVE_LOCALVEROCTFILEDIR "${_octave_config_localveroctfiledir}" )

endif(OCTAVE_CONFIG_EXECUTABLE)


MARK_AS_ADVANCED(
    OCTAVE_LIBRARY_FOUND
    OCTAVE_CXXFLAGS
    OCTAVE_LINK_FLAGS
    OCTAVE_INCLUDE_DIRS
    OCTAVE_LINK_DIRS
    OCTAVE_LIBRARY
    OCTAVE_LIBRARY_RELEASE
    OCTAVE_LIBRARY_DEBUG
)