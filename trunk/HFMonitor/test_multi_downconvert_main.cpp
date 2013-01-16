// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "multi_downconvert_processor.hpp"
#include "processor/registry.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_config_multi_downconvert");
  try {
    const std::string filename((argc > 1 ) ? argv[1] : "config_multi_downconverter.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    const std::string stream_name("DataIQ");

    client<iq_adapter<repack_processor<multi_downconvert_processor<double> > > >
      c(config.get_child("MultiDownConverter"));
    
    
    const std::set<std::string> streams(c.ls());
    if (streams.find(stream_name) != streams.end())
      ASSERT_THROW(c.connect_to(stream_name) == true);
    else
      throw std::runtime_error(str(boost::format("stream '%s' is not available")
                                   % stream_name));
    c.start();
    run_in_thread(network::get_io_service());
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
