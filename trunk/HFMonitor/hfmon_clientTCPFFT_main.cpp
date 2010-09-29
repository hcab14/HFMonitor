// -*- C++ -*-
// $Id$
#include <iostream>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>

#include "protocol.hpp"
#include "run.h"

#include "ClientTCP.hpp"

#include "Raw2IQAdapter.hpp"
#include "RepackProcessor.hpp"
#include "FFTProcessor.hpp"

int main(int argc, char* argv[])
{
  using boost::asio::ip::tcp;
  try {
    if (argc != 3) {
      std::cerr << "Usage: client <host> <port>" << std::endl;
      return 1;
    }   
    boost::asio::io_service io_service;
    
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(argv[1], argv[2]);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    boost::property_tree::ptree config;
    config.put("SamplesPerFile", 1000u*1000u);
    config.put("FileNamePattern", std::string("data_%08X.bin"));
    config.put("Repack.BufferLengthSec", 1.0);

    ClientTCP<Raw2IQAdapter<RepackProcessor<FFTProcessor> > > c(io_service, endpoint_iterator, config);

    run(io_service, c);

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }  
  return 0;
}
