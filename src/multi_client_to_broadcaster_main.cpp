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
#include "demod_alpha_processor.hpp"
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

/*! \addtogroup executables
 *  @{
 * \addtogroup multi_client_to_broadcaster multi_client_to_broadcaster
 * multi_client_to_broadcaster
 * - @ref network::client::multi_client with output to @ref network::broadcaster::broadcaster
 * - configuration using command-line / XML
 * 
 * @{
 */

namespace network {
  namespace client {
    /// @ref network::client::multi_client with output to @ref network::broadcaster::broadcaster
    class multi_client_to_broadcaster : public multi_client {
    public:
      typedef boost::property_tree::ptree ptree;
      multi_client_to_broadcaster(const ptree& config)
        : multi_client(config)
        , broadcaster_(broadcaster::broadcaster::make(config.get_child("Broadcaster")))
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
      broadcaster::broadcaster::sptr broadcaster_;
      bool         started_;
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

    FFT::FFTWInitThreads fftwInit;

    processor::registry::add<writer_txt>("WriterTXT");
    processor::registry::add<network::iq_adapter<wave::writer_iq>     >("WriterIQ");
    processor::registry::add<network::iq_adapter<FFTProcessor<float > > >("FFTProcessor_FLOAT");
    processor::registry::add<network::iq_adapter<FFTProcessor<double> > >("FFTProcessor_DOUBLE");
    processor::registry::add<network::iq_adapter<demod_msk_processor  > >("DemodMSK");
    processor::registry::add<network::iq_adapter<demod_fsk_processor  > >("DemodFSK");
    processor::registry::add<network::iq_adapter<demod_alpha_processor> >("DemodALPHA");

    network::client::multi_client_to_broadcaster c(config_multi_client);

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
/// @}
/// @}
