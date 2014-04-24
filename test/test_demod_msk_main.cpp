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
#include <boost/property_tree/xml_parser.hpp>

#include "demod_msk_processor.hpp"
#include "wave/reader.hpp"
#include "run.hpp"


int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_demod_msk");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config/demod_msk.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    wave::reader_iq<demod_msk_processor> r(config.get_child("DemodMSK"));

    r.process_file("DataBC/DC_023400_DH0/y2013-m02-d14_H12M50.wav");
    r.finish();

  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
