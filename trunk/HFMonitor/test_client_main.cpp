// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include "network/client.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_client");
  try {
    // const std::string filename((argc > 1 ) ? argv[1] : "config_client.xml");
    // boost::property_tree::ptree config;
    // read_xml(filename, config);

    const std::string server_name = "localhost";
    const std::string server_port = "18001";
    const std::string stream_name = "DataIQ";

    boost::asio::io_service io_service;

    client c(io_service, server_name, server_port);
    const broadcaster_directory::sptr bd(c.ls());
    if (bd->contains(stream_name))
      c.connect_to(stream_name);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    
    io_service.run();
    for (size_t i(0); i<10; ++i) {
      std::cout << "." << std::endl;
      sleep(1);
    }
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
