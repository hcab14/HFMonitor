// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "FFTProcessor.hpp"
#include "demod_fsk_processor.hpp"
#include "demod_msk_processor.hpp"
#include "network.hpp"
#include "network/iq_adapter.hpp"
#include "network/multi_client.hpp"
#include "processor/registry.hpp"
#include "run.hpp"
#include "wave/writer.hpp"
#include "writer.hpp"

#include "station_info.hpp"

namespace network {
  namespace client {
    class multi_client_tofile : public multi_client, public writer_txt_base {
    public:
      typedef boost::property_tree::ptree ptree;
      multi_client_tofile(const ptree& config)
        : multi_client(config)
        , writer_txt_base(config.get_child("FileSink"))
        , station_info_(config.get<std::string>("StationInfo")) {}

      virtual processor::result_base::sptr dump(processor::result_base::sptr rp) {
        // result -> file
        // std::cout << "DUMP: " << rp->name() << " " << rp->to_string() << std::endl;
        return dump_result(rp, station_info_.to_string());
      }

    protected:
    private:
      station_info station_info_;
    } ;

  } // namespace client
} // namespace network

int main(int argc, char* argv[])
{
  boost::program_options::variables_map vm;
  try {
    vm = process_options("config/multi_client.xml", argc, argv);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  boost::filesystem::path p(vm["config"].as<std::string>());
  LOGGER_INIT("./Log", p.stem().native());
  try {
    typedef boost::property_tree::ptree ptree;
    ptree config;
    read_xml(vm["config"].as<std::string>(), config, boost::property_tree::xml_parser::no_comments);
    const ptree& config_multi_client(config.get_child("MultiClient"));

    // fill spm (stream name -> processor name) multimap
    network::client::multi_client::stream_processor_map spm;
    BOOST_FOREACH(const ptree::value_type& s,
                  config_multi_client.get_child("Streams")) {
      if (s.first != "Stream") {
        LOG_INFO(str(boost::format("ignoring invalid key: '%s'") % s.first));
        continue;
      }
      spm.insert(std::make_pair(s.second.get<std::string>("<xmlattr>.pattern"), s.second.get<std::string>("")));
    }

    processor::registry::add<writer_txt>("WriterTXT");
    processor::registry::add<network::iq_adapter<wave::writer_iq      > >("WriterIQ");
    processor::registry::add<network::iq_adapter<FFTProcessor<float > > >("FFTProcessor_FLOAT");
    processor::registry::add<network::iq_adapter<FFTProcessor<double> > >("FFTProcessor_DOUBLE");
    processor::registry::add<network::iq_adapter<demod_msk_processor>   >("DemodMSK");
    processor::registry::add<network::iq_adapter<demod_fsk_processor  > >("DemodFSK");

    network::client::multi_client_tofile c(config_multi_client);

    const std::set<std::string> streams(c.ls());
    BOOST_FOREACH(std::string stream, streams)
      std::cout << "-- " << stream << std::endl;

    ASSERT_THROW(c.connect_to(spm) == true);

    c.start();
    run_in_thread(network::get_io_service());
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
