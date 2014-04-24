// -*- C++ -*-
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

#include <algorithm>
#include <complex>
#include <iostream>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <bzlib.h>

#include "carrier_monitoring/background_estimator.hpp"
#include "FFT.hpp"
#include "FFTProcessor/Filter.hpp"
#include "network.hpp"
#include "network/client.hpp"
#include "network/iq_adapter.hpp"
#include "repack_processor.hpp"
#include "run.hpp"
#include "Spectrum.hpp"

#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <vector>
#include <cmath>


class single_channel_carrier_monitor : public boost::noncopyable {
public:
  typedef boost::shared_ptr<single_channel_carrier_monitor> sptr;

  typedef frequency_vector<double> fvector;
  typedef fvector::iterator fvector_iterator;
  typedef fvector::const_iterator fvector_const_iterator;

  virtual ~single_channel_carrier_monitor() {}

  static sptr make(std::string type, double fCenter_kHz, double fMin_kHz, double fMax_kHz) {
    return sptr(new single_channel_carrier_monitor(type, fCenter_kHz, fMin_kHz, fMax_kHz));
  }
  template<typename FFT_SPECTRUM_TYPE>
  void process(FFT_SPECTRUM_TYPE& s, processor::service_iq::sptr sp) {

    const fvector ps(fMin_Hz_, fMax_Hz_, s, std::abs<double>, (sp ? sp->offset_ppb() : 0.));
    if (filter_.x().empty())
      filter_.init(sp->approx_ptime(), ps);
    else
      filter_.update(sp->approx_ptime(), ps);

    // estimate background and compute a background subtracted spectrum
    fvector xf(filter_.x());
    const unsigned poly_degree(2);
    const size_t num_intervals(15);
    background_estimator bg(xf.size(), poly_degree, num_intervals);

    const double fit_threshold_db(2.5);
    if (false == bg.do_fit(xf, fit_threshold_db))
      return;
    const char   spec_threshold_db(2);
    const double spec_max_occupancy(0.08);
    const std::pair<vector_compressor::vector_type, char> pp(bg.make_spectrum(spec_threshold_db, spec_max_occupancy));
    std::cout << "threshold,occupancy = " << int(pp.second) << " " << double(pp.first.size())/double(xf.size()) << std::endl;

    // find peaks
    const double threshold_db(5. + pp.second - spec_threshold_db);
    std::cout << str(boost::format("%s %8.1f %s max: ")
		     % type_
		     % (1e-3*fCenter_Hz_)
		     % sp->approx_ptime());
    for (size_t j(0); j<10; ++j) {
      fvector_iterator i(std::max_element(xf.begin()+10, xf.end()-10, compare_second));
      const size_t index(std::distance(xf.begin(), i));
      if (bg.spec_db(index)-bg.spec_db_fitted(index) < threshold_db)
	break;
      const std::pair<double, bool> f(estimate_peak(xf.begin()+10, xf.end()-10, i));
      if (f.second)
	std::cout << str(boost::format("[%.1f %.0f %.0f] ") 
			 % f.first
			 % bg.spec_db(index)
			 % (bg.spec_db(index)-bg.spec_db_fitted(index)));
    }
    std::cout << std::endl;    
  }
protected:
  std::pair<double, bool>
  estimate_peak(fvector_iterator beg,
		fvector_iterator end,
		fvector_iterator i_peak) const {
    
    const double s_peak(i_peak->second);
    double sum_w(i_peak->second);
    double sum_wx(i_peak->second*i_peak->first);
    i_peak->second = 0.0;
    
    size_t n_plus(0), n_minus(0);
    double s(s_peak);
    for (fvector_iterator i(i_peak+1); i->second < s && i->second != 0 && i < end; ++i) {
//       std::cout << str(boost::format("\n(+): %.1f (%.2e %.2e)\n") % i->first  % s % i->second);
      s=i->second;
      sum_w  += i->second;
      sum_wx += i->second*i->first;
      i->second = 0.0;
      ++n_plus;
    }
    s = s_peak;
    for (fvector_iterator i(i_peak-1); i->second < s && i->second != 0 && i > beg; --i) {
//       std::cout << str(boost::format("\n(-): %.1f (%.2e %.2e)\n") % i->first  % s % i->second);
      s=i->second;
      sum_w  += i->second;
      sum_wx += i->second*i->first;
      i->second = 0.0;
      ++n_minus;
    }
    return std::make_pair((sum_w != 0) ? sum_wx / sum_w : 0, n_plus>0 && n_minus>0);
  }

  static bool compare_second(const std::pair<double,double>& a,
			     const std::pair<double,double>& b) {
    return a.second < b.second;
  }

private:
  single_channel_carrier_monitor(std::string type, double fCenter_kHz, double fMin_kHz, double fMax_kHz)
    : type_(type)
    , fCenter_Hz_(1e3*fCenter_kHz)
    , fMin_Hz_(1e3*fMin_kHz)
    , fMax_Hz_(1e3*fMax_kHz)
    , spec_counter_(0) {
    filter_.add(Filter::LowPass<frequency_vector<double> >::make(1.0, 15));
  }

  const std::string type_;
  const double fCenter_Hz_;
  const double fMin_Hz_;
  const double fMax_Hz_;
  size_t spec_counter_;
  Filter::Cascaded<frequency_vector<double> > filter_;
} ;

class carrier_monitor_processor : public processor::base_iq {
public:
  typedef FFT::FFTWTransform<double> fft_type;
  typedef std::vector<single_channel_carrier_monitor::sptr> channel_vector_type;

  carrier_monitor_processor(const ptree& config)
    : processor::base_iq(config)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE) {
    const ptree& config_channels(config.get_child("Channels"));
    BOOST_FOREACH(const ptree::value_type& channel_type, config_channels) {
      std::cout << "type= " << channel_type.first << std::endl;
      const double fMin_kHz  (channel_type.second.get<double>("<xmlattr>.fMin_kHz"));
      const double fStep_kHz (channel_type.second.get<double>("<xmlattr>.fStep_kHz"));
      const double fMax_kHz  (channel_type.second.get<double>("<xmlattr>.fMax_kHz"));
      const double fWidth_kHz(channel_type.second.get<double>("<xmlattr>.fWidth_kHz"));
      for (double f_kHz(fMin_kHz); f_kHz<fMax_kHz+1e-9; f_kHz+=fStep_kHz)
	channels_.push_back
	  (single_channel_carrier_monitor::make
	   (channel_type.first, f_kHz, f_kHz-0.5*fWidth_kHz, f_kHz+0.5*fWidth_kHz));
    }
  }
  virtual ~carrier_monitor_processor(){}

  void process_iq(service::sptr sp, const_iterator i0, const_iterator i1) {
    const size_t length(std::distance(i0, i1));
#if 0
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
	      << " length=" << length
              << std::endl;
#endif
    // (1) compute FFT spectrum
    if (length != fftw_.size())
      fftw_.resize(length);
    fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<double>(length));
    const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    
    // (2) process each channel
    BOOST_FOREACH(single_channel_carrier_monitor::sptr channel, channels_)
      channel->process(s, sp);
  }

private:
  fft_type fftw_;
  channel_vector_type channels_;
} ;

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?",                                                       "produce help message")
    ("version,v",                                                    "display version")
    ("host,h", po::value<std::string>()->default_value("127.0.0.1"), "server hostname")
    ("port,p", po::value<std::string>()->default_value("18001"),     "server port")
    ("stream,s", po::value<std::string>()->default_value("DataIQ"),  "stream name")
    ("overlap,o", po::value<double>()->default_value(0.),       "buffer overlap")
    ("delta_t,dt", po::value<double>()->default_value(1.),      "dt (sec)")
    ("fMin",    po::value<double>()->default_value(1413.0),    "fMin   (kHz)")
    ("fStep",   po::value<double>()->default_value(   9.0),    "fStep  (kHz)")
    ("fMax",    po::value<double>()->default_value(1611.0),    "fMax   (kHz)")
    ("fWidth",  po::value<double>()->default_value(   0.2),    "fWidth (kHz)")
    ;

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;
    }
    if (vm.count("version")) {
      std::cout << SVN_VERSION_STRING << std::endl;
      return 1;
    }
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }
  
  LOGGER_INIT("./Log", "carrier_monitor");

  try {
    // make up ptree config
    boost::property_tree::ptree config;
    config.put("server.<xmlattr>.host", vm["host"].as<std::string>());
    config.put("server.<xmlattr>.port", vm["port"].as<std::string>());
    config.put("Repack.<xmlattr>.bufferLength_sec", vm["delta_t"].as<double>());
    config.put("Repack.<xmlattr>.overlap_percent", vm["overlap"].as<double>());
    config.put("Channels.EU.<xmlattr>.fMin_kHz",   vm["fMin"].as<double>());
    config.put("Channels.EU.<xmlattr>.fStep_kHz",  vm["fStep"].as<double>());
    config.put("Channels.EU.<xmlattr>.fMax_kHz",   vm["fMax"].as<double>());
    config.put("Channels.EU.<xmlattr>.fWidth_kHz", vm["fWidth"].as<double>());

    const std::string stream_name(vm["stream"].as<std::string>());
    client<iq_adapter<repack_processor<carrier_monitor_processor> > > c(config);

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
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
