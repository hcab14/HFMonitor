// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "FFTProcessorToBC.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "processor/registry.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "client_FFTProcessorToBC");
  try {
    const boost::program_options::variables_map vm(process_options("config/FFTProcessor.xml", argc, argv));
    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config);

    processor::registry::add<FFTProcessorToBC<float>  >("FFTProcessorToBC_FLOAT");
    processor::registry::add<FFTProcessorToBC<double> >("FFTProcessorToBC_DOUBLE");

    const std::string stream_name("DataIQ");

    client<iq_adapter<repack_processor<FFTProcessorToBC<double> > > >
      c(config.get_child("FFTProcessor"));

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
