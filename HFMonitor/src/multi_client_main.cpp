// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include <boost/property_tree/xml_parser.hpp>
#include <iostream>

#include "FFTProcessorToBC.hpp"
#include "FFTProcessorToFile.hpp"
#include "network.hpp"
#include "network/iq_adapter.hpp"
#include "network/multi_client.hpp"
#include "processor/registry.hpp"
#include "run.hpp"
#include "wave/writer.hpp"
#include "writer.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "multi_client");
  try {
    const boost::program_options::variables_map vm(process_options("config/multi_client.xml", argc, argv));
    typedef boost::property_tree::ptree ptree;
    ptree config;
    read_xml(vm["config"].as<std::string>(), config);
    const ptree& config_multi_client(config.get_child("MultiClient"));
    const ptree& config_clients(config.get_child("MultiClient.Clients"));

    // fill spm
    multi_client::stream_processor_config_map spcm;
    BOOST_FOREACH(const ptree::value_type& s,
                  config_multi_client.get_child("Streams")) {
      if (s.first != "Stream") {
        LOG_INFO(str(boost::format("ignoring invalid key: '%s'") % s.first));
        continue;
      }
      const std::string pattern(s.second.get<std::string>("<xmlattr>.pattern"));
      const std::string name(s.second.get<std::string>(""));
      const std::string config_key(std::string("Clients.")+name);
      const ptree::const_assoc_iterator i(config_clients.find(name));
      if (i == config_clients.not_found()) {
        LOG_INFO(str(boost::format("configuration for processor '%s' not found") % config_key));
        continue;
      }
      const std::string type(i->second.get<std::string>("<xmlattr>.type"));
      LOG_INFO(str(boost::format("registering processor '%s' for steam(s) '%s' with config key '%s' of type '%s'") % name % pattern % config_key % type));
      spcm[pattern] = std::make_pair(type, config_key);
    }

    processor::registry::add<writer_txt                  >("WriterTXT");
    processor::registry::add<iq_adapter<wave::writer_iq> >("WriterIQ");
    processor::registry::add<FFTProcessorToBC<float>     >("FFTProcToBC_FLOAT");
    processor::registry::add<FFTProcessorToBC<double>    >("FFTProcToBC_DOUBLE");

    multi_client c(config_multi_client);

    const std::set<std::string> streams(c.ls());
    BOOST_FOREACH(std::string stream, streams)
      std::cout << "-- " << stream << std::endl;

    ASSERT_THROW(c.connect_to(spcm) == true);

    c.start();
    run_in_thread(network::get_io_service());
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
