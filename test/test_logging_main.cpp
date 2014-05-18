// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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
  logging::init("./Log", "test_logging");

  try {
    const boost::posix_time::ptime tag_i(boost::posix_time::microsec_clock::universal_time());
    LOG_INFO("Info");
    usleep(10*1000);
    const boost::posix_time::ptime tag_e(boost::posix_time::microsec_clock::universal_time());
    LOG_ERROR("Error");
    usleep(10*1000);
    const boost::posix_time::ptime tag_s(boost::posix_time::microsec_clock::universal_time());
    LOG_STATUS("Status");
    usleep(10*1000);
    const boost::posix_time::ptime tag_w(boost::posix_time::microsec_clock::universal_time());
    LOG_WARNING("Warning");
    usleep(10*1000);

    LOG_INFO_T(tag_i, "Info");
    LOG_ERROR_T(tag_e, "Error");
    LOG_STATUS_T(tag_s, "Status");
    LOG_WARNING_T(tag_w, "Warning");
    throw std::runtime_error("Hello");
  } catch (const std::runtime_error& e) {
    LOG_ERROR("EXEPTION");
    LOGGER_ERROR << e.what();
  }
}
