// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

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
