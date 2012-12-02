// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include "network/client_iq.hpp"
#include "repack_processor.hpp"

class test_proc {
public:
  test_proc(const boost::property_tree::ptree&) {}

  void process_iq(processor::service_iq::sptr sp,
                  std::vector<std::complex<double> >::const_iterator i0,
                  std::vector<std::complex<double> >::const_iterator i1) {
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << std::endl;
  }
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_client");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config_client.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    boost::asio::io_service io_service;

    const std::string stream_name("DataIQ");

    client_iq<repack_processor<test_proc> > c(io_service, config.get_child("server"));
    const broadcaster_directory::sptr bd(c.ls());
    if (bd->contains(stream_name))
      c.connect_to(stream_name);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    
    io_service.run();
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
