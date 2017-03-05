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
#include "network/broadcaster.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "multi_downconvert_processor.hpp"
#include "processor/registry.hpp"
#include "repack_processor.hpp"
#include "network/broadcaster.hpp"
#include "run.hpp"
#include "tracking_goertzel_processor.hpp"
#include "wave/writer.hpp"
#include "writer.hpp"

#include "network/protocol.hpp"
#include "wave/writer.hpp"
#include "station_info.hpp"

#include "filter/fir/overlap_save.hpp"
#include "cl/fft/overlap_save.hpp"

/*! \addtogroup executables
 *  @{
 * \addtogroup client_multi_downconvert client_multi_downconvert
 * client_multi_downconvert
 * - @ref multi_downconvert_processor with output to @ref network::broadcaster::broadcaster
 * - configuration using command-line / XML
 *
 * @{
 */

/// adaptation of @ref multi_downconvert_processor to output to @ref network::broadcaster::broadcaster
///
template<class OS_SETUP>
class multi_downconvert_toBC : public multi_downconvert_processor<OS_SETUP> {
public:
  typedef boost::shared_ptr<multi_downconvert_toBC<OS_SETUP> > sptr;

  typedef typename multi_downconvert_processor<OS_SETUP>::service service;
  typedef typename multi_downconvert_processor<OS_SETUP>::const_iterator const_iterator;

  multi_downconvert_toBC(const boost::property_tree::ptree& config)
    : multi_downconvert_processor<OS_SETUP>(config)
    , broadcaster_(network::broadcaster::broadcaster::make(config.get_child("Broadcaster")))
    , started_(false)
    , station_info_(config.get<std::string>("StationInfo")) {}

  virtual ~multi_downconvert_toBC() {}

protected:
  virtual void dump(typename service::sptr sp, const_iterator begin, const_iterator end) {
    if (not started_) {
      broadcaster_->start();
      started_= true;
    }
    const size_t bytes_per_sample(3);
    // data
    std::string s(2*bytes_per_sample*std::distance(begin,end), 0);
    std::string::iterator si(s.begin());
    for (auto i(begin); i!=end; ++i) {
      si = wave::detail::write_real_sample(si, 8*bytes_per_sample, i->real());
      si = wave::detail::write_real_sample(si, 8*bytes_per_sample, i->imag());
    }
    const network::protocol::iq_info h_iq
      (sp->sample_rate_Hz(),
       sp->center_frequency_Hz(),
       'I', bytes_per_sample, sp->offset_ppb(), sp->offset_ppb_rms());
    std::string data(sizeof(network::protocol::iq_info)+s.size(), 0);
    std::copy(h_iq.begin(), h_iq.end(), data.begin());
    std::copy(s.begin(), s.end(), data.begin()+sizeof(network::protocol::iq_info));
    broadcaster_->bc_data(sp->approx_ptime(), sp->stream_name(), "WAV_0000", data);
  }


  virtual processor::result_base::sptr dump(processor::result_base::sptr rp) {
    LOG_INFO(str(boost::format("dump: %s") % *rp));
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

private:
  network::broadcaster::broadcaster::sptr broadcaster_;
  bool              started_;
  station_info      station_info_;
} ;

int main(int argc, char* argv[])
{
  boost::program_options::variables_map vm;
  try {
    vm = process_options("config/multi_downconvert.xml", argc, argv);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  boost::filesystem::path p(vm["config"].as<std::string>());
  LOGGER_INIT("./Log", p.stem().native());
  try {
    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config, boost::property_tree::xml_parser::no_comments);

    FFT::FFTWInitThreads fftwInit;

    processor::registry::add<writer_txt>("WriterTXT");
    processor::registry::add<network::iq_adapter<wave::writer_iq      > >("WriterIQ");
    processor::registry::add<network::iq_adapter<FFTProcessor<float > > >("FFTProcessor_FLOAT");
//     processor::registry::add<network::iq_adapter<FFTProcessor<double> > >("FFTProcessor_DOUBLE");
    processor::registry::add<tracking_goertzel_processor>("TrackingGoertzel");

    const std::string stream_name(config.get<std::string>
                                  ("MultiDownConverter.server.<xmlattr>.stream_name", "DataIQ"));

    // typedef filter::fir::overlap_save_setup  os_setup_type;
    typedef cl::fft::overlap_save_setup os_setup_type;
    network::client::client<network::iq_adapter<repack_processor<multi_downconvert_toBC<os_setup_type> > > >
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
/// @}
/// @}
