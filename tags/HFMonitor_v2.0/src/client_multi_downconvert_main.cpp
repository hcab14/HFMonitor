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
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

#include "FFTProcessorToBC.hpp"
#include "FFTProcessorToFile.hpp"
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

template<typename FFTFloat>
class multi_downconvert_toBC : public multi_downconvert_processor<FFTFloat> {
public:
  typedef boost::shared_ptr<multi_downconvert_toBC<FFTFloat> > sptr;
  typedef typename multi_downconvert_processor<FFTFloat>::ptree ptree;
  typedef typename multi_downconvert_processor<FFTFloat>::service service;
  typedef typename multi_downconvert_processor<FFTFloat>::filter_param filter_param;
  typedef typename multi_downconvert_processor<FFTFloat>::overlap_save_type overlap_save_type;

  multi_downconvert_toBC(const boost::property_tree::ptree& config)
    : multi_downconvert_processor<FFTFloat>(config)
    , broadcaster_(broadcaster::make(config.get_child("Broadcaster")))
    , started_(false)
    , station_info_(config.get<std::string>("StationInfo")) {}

  virtual ~multi_downconvert_toBC() {}

protected:
  virtual void dump(typename service::sptr sp,
                    const filter_param& fp,
                    const typename overlap_save_type::complex_vector_type& out) {
    if (not started_) {
      broadcaster_->start();
      started_= true;
    }
    LOG_INFO(str(boost::format("multi_downconvert_toBC::dump  '%s' size=%d")
                 % fp.name() % out.size()));

    typedef typename overlap_save_type::complex_vector_type complex_vector_type;
    const size_t bytes_per_sample(3);
    // data
    std::ostringstream oss;
    for (typename complex_vector_type::const_iterator i(out.begin()); i!=out.end(); ++i) {
      wave::detail::write_real_sample(oss, 8*bytes_per_sample, i->real());
      wave::detail::write_real_sample(oss, 8*bytes_per_sample, i->imag());
    }
    iq_info h_iq(sp->sample_rate_Hz(),
                 sp->center_frequency_Hz(),
                 'I', bytes_per_sample, sp->offset_ppb(), sp->offset_ppb_rms());
    std::string data(sizeof(iq_info)+oss.str().size(), 0);
    std::copy(h_iq.begin(), h_iq.end(), data.begin());
    const std::string& s(oss.str());
    std::copy(s.begin(), s.end(), data.begin()+sizeof(iq_info));
    broadcaster_->bc_data(sp->approx_ptime(), fp.name(), "WAV_0000", data);
  }
  

  virtual processor::result_base::sptr dump(processor::result_base::sptr rp) {
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
  broadcaster::sptr broadcaster_;
  bool              started_;
  station_info      station_info_;
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "multi_downconvert_");
  try {
    const boost::program_options::variables_map
      vm(process_options("config/multi_downconvert.xml", argc, argv));
    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config);

    processor::registry::add<writer_txt                  >("WriterTXT");
    processor::registry::add<iq_adapter<wave::writer_iq> >("WriterIQ");
    processor::registry::add<FFTProcessorToBC<float>     >("FFTProcToBC_FLOAT");
    processor::registry::add<FFTProcessorToBC<double>    >("FFTProcToBC_DOUBLE");

    processor::registry::add<tracking_goertzel_processor >("TrackingGoertzel");

    const std::string stream_name("DataIQ");

    client<iq_adapter<repack_processor<multi_downconvert_toBC<double> > > >
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
