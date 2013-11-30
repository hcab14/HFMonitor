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
#include <boost/property_tree/xml_parser.hpp>
#include <iostream>

#include "FFTProcessor.hpp"
#include "demod_fsk_processor.hpp"
#include "demod_msk_processor.hpp"
#include "network.hpp"
#include "network/broadcaster.hpp"
#include "network/iq_adapter.hpp"
#include "network/multi_client.hpp"
#include "processor/registry.hpp"
#include "run.hpp"
#include "wave/writer.hpp"
#include "writer.hpp"
#include "station_info.hpp"

class multi_client_to_broadcaster : public multi_client {
public:
  typedef boost::property_tree::ptree ptree;
  multi_client_to_broadcaster(const ptree& config)
    : multi_client(config)
    , broadcaster_(broadcaster::make(config.get_child("Broadcaster")))
    , started_(false)
    , station_info_(config.get<std::string>("StationInfo")) {}

  virtual ~multi_client_to_broadcaster() {}

protected:
  virtual processor::result_base::sptr dump(processor::result_base::sptr rp) {
//     LOG_INFO(str(boost::format("multi_client_to_broadcaster::dump: %s %s") % rp->name() % rp->to_string()));
    if (not started_) {
      broadcaster_->start();
      started_= true;
    }

    std::ostringstream oss_header;
    oss_header << station_info_;
    rp->dump_header(oss_header);
    std::string header_str(oss_header.str());

    std::ostringstream oss_data;
    rp->dump_data(oss_data);
    std::string data_str(oss_data.str());
    
    broadcaster_->bc_data(rp->approx_ptime(), rp->name(), rp->format(), data_str, header_str);
    return rp;
  }

  virtual void dump_wav(processor::service_base::ptime t, std::string str_name,
                        data_buffer_type::const_iterator begin,
                        data_buffer_type::const_iterator end) {
    std::string data(begin, end);
    broadcaster_->bc_data(t, str_name, "WAV_0000", data);
  }

private:
  broadcaster::sptr broadcaster_;
  bool              started_;
  station_info      station_info_;
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "multi_client_to_broadcaster");
  try {
    const boost::program_options::variables_map vm(process_options("config/multi_client.xml", argc, argv));
    typedef boost::property_tree::ptree ptree;
    ptree config;
    read_xml(vm["config"].as<std::string>(), config, boost::property_tree::xml_parser::no_comments);
    const ptree& config_multi_client(config.get_child("MultiClient"));
    const ptree& config_clients(config.get_child("MultiClient.Clients"));

    // fill spm
    multi_client_to_broadcaster::stream_processor_config_map spcm;
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
      spcm.insert(std::make_pair(pattern, std::make_pair(type, config_key)));
    }

    processor::registry::add<writer_txt>("WriterTXT");
    processor::registry::add<iq_adapter<wave::writer_iq>     >("WriterIQ");
    processor::registry::add<iq_adapter<FFTProcessor<float > > >("FFTProcessor_FLOAT");
    processor::registry::add<iq_adapter<FFTProcessor<double> > >("FFTProcessor_DOUBLE");
    processor::registry::add<iq_adapter<demod_msk_processor  > >("DemodMSK");
    processor::registry::add<iq_adapter<demod_fsk_processor  > >("DemodFSK");

    multi_client_to_broadcaster c(config_multi_client);

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
