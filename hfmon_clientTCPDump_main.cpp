// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "protocol.hpp"
#include "run.hpp"

#include "ClientTCP.hpp"
#include "DumpProcessor.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", argv[0]);
  using boost::asio::ip::tcp;
  try {
    if (argc < 3) {
      std::cerr << "Usage: client <host> <port>" << std::endl;
      return 1;
    }   
    boost::asio::io_service io_service;
    
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(argv[1], argv[2]);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    boost::property_tree::ptree config;
    std::string filename((argc > 3 ) ? argv[3] : "config_DumpProcessor.xml");
    read_xml(filename, config);    

    ClientTCP<DumpProcessor> c(io_service, endpoint_iterator, config.get_child("DumpProcessor"));
    run(io_service, c);

  } catch (const std::exception& e) {
    LOG_ERROR(e.what());
    std::cerr << e.what() << std::endl;
    return 1;
  }  
  return 0;
}
