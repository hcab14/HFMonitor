// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <iostream>
#include "logging.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_logging");

  try {
    LOG_INFO("Info");
    LOG_ERROR("Info");
    throw std::runtime_error("Hello");
  } catch (const std::runtime_error& e) {
    LOGGER_ERROR << e.what() << " 2nd arg" << " 3rd arg";
  }
}
