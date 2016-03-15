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
#include <cmath>
#include <deque>
#include <iostream>
#include <fstream>
#include <iterator>
#include <map>
#include <numeric>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "logging.hpp"
#include "wave/reader.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

#include "demod_alpha_processor.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_alpha");
  try {
    const boost::program_options::variables_map
      vm(process_options("config/multi_downconvert.xml", argc, argv));

    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config);

    wave::reader_iq<demod_alpha_processor> r(config.get_child("Test"));
    for (int i((argc == 1) ? 1 : 3); i<argc; ++i) {
      std::cout << "processing " << argv[i] << std::endl;
      r.process_file(argv[i]);
    }
    r.finish();

  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    LOG_ERROR(e.what()); 
    return 1;
  }
  return 0;
}

