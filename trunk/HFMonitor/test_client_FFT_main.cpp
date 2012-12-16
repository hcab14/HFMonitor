// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "FFTProcessor.hpp"
#include "network/client_iq.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_client");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config_client.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    boost::asio::io_service io_service;

    const std::string stream_name("DataIQ");

    client_iq<repack_processor<FFTProcessor<double> > > c(io_service, config.get_child("FFTProcessor"));
    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      c.connect_to(stream_name);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    
    run_in_thread(io_service);
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
