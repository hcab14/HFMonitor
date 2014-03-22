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

#include "carrier_monitoring/polynomial_interval_fit.hpp"
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

bool compare_second(std::pair<double,double>& a,
		    std::pair<double,double>& b) {
  return a.second < b.second;
}

class carrier_monitor_processor {
public:
  typedef FFT::FFTWTransform<double> fft_type;

  typedef frequency_vector<double> fvector;
  typedef fvector::iterator fvector_iterator;

  carrier_monitor_processor(const boost::property_tree::ptree& config)
    : fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , host_(config.get<std::string>("server.<xmlattr>.host"))
    , port_(config.get<std::string>("server.<xmlattr>.port"))
    , fmin_Hz_(1e3*config.get<double>(".<xmlattr>.fMin_kHz"))
    , fmax_Hz_(1e3*config.get<double>(".<xmlattr>.fMax_kHz"))
    , spec_counter_(0) {
    filter_.add(Filter::LowPass<frequency_vector<double> >::make(1.0, 15));
  }

  void process_iq(processor::service_iq::sptr sp,
                  std::vector<std::complex<double> >::const_iterator i0,
                  std::vector<std::complex<double> >::const_iterator i1) {
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
    if (length != fftw_.size())
      fftw_.resize(length);
    fftw_.transformRange(i0, i1, FFT::WindowFunction::Blackman<double>(length));
    const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    fvector ps(fmin_Hz_, fmax_Hz_, s, std::abs<double>, (sp ? sp->offset_ppb() : 0.));
    if (filter_.x().empty())
      filter_.init(sp->approx_ptime(), ps);
    else
      filter_.update(sp->approx_ptime(), ps);

    fvector xf(filter_.x());

    // 
    const double threshold_db(10.);
    const size_t n(xf.size());
    std::vector<double> v(n, 0);
    std::vector<size_t> b(n, 1);
    for (size_t i(0); i<n; ++i)
      v[i] = 20*log10(xf[i].second);

    const unsigned poly_degree(2);
    std::vector<size_t> indices;
    for (size_t i=0; i<20; ++i)
      indices.push_back((i*n)/20);
    indices.push_back(n-1);
    polynomial_interval_fit p(poly_degree, indices);

    for (size_t l(0); l<20; ++l) {
      if (!p.fit(v, b)) {
	std::cerr << "fit failed" << std::endl;
      }
      size_t nchanged(0);
      for (size_t i(0); i<n; ++i) {
	const std::pair<double,double> vf(p.eval(i));
	const bool c(v[i]-vf.first > 5.);
	nchanged += (c==b[i]);
	b[i] = !c;
      }
      if (0 == nchanged)
	break;
    }

#if 0
    //
    for (fvector_iterator i(xf.begin()), end(xf.end()); i!=end; ++i) {
      const size_t index(std::distance(xf.begin(), i));
      const std::pair<double,double> vf(p.eval(index));
      std::cout << str(boost::format("S%06d %.1f %.3e %5.1f %5.1f\n") % spec_counter_ % i->first % i->second % v[index] % vf.first);
    }
#endif
    ++spec_counter_;

    std::cout << str(boost::format("%s max: ")  % sp->approx_ptime());
    for (size_t j(0); j<10; ++j) {
      fvector_iterator i(std::max_element(xf.begin()+10, xf.end()-10, compare_second));
      const size_t index(std::distance(xf.begin(), i));
      const std::pair<double,double> vf(p.eval(index));
      if (v[index]-vf.first < threshold_db)
	break;
      const std::pair<double, bool> f(estimate_peak(xf.begin(), xf.end(), i));
      if (f.second)
	std::cout << str(boost::format("[%.1f %.0f %.0f] ") % f.first % v[index] % (v[index]-vf.first));
    }
    std::cout << std::endl;

//     w_.get_spec_display().insert_spec(ps.apply(s2db()), xf.apply(s2db()), sp);
  }

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
    for (fvector_iterator i(i_peak+1); i->second < s && i->second != 0 && i != end; ++i) {
//       std::cout << str(boost::format("\n(+): %.1f (%.2e %.2e)\n") % i->first  % s % i->second);
      s=i->second;
      sum_w  += i->second;
      sum_wx += i->second*i->first;
      i->second = 0.0;
      ++n_plus;
    }
    s = s_peak;
    for (fvector_iterator i(i_peak-1); i->second < s && i->second != 0 && i != beg; --i) {
//       std::cout << str(boost::format("\n(-): %.1f (%.2e %.2e)\n") % i->first  % s % i->second);
      s=i->second;
      sum_w  += i->second;
      sum_wx += i->second*i->first;
      i->second = 0.0;
      ++n_minus;
    }
    return std::make_pair((sum_w != 0) ? sum_wx / sum_w : 0, n_plus>0 && n_minus>0);
  }
private:
  struct s2db {
    double operator()(double c) const {
      return 20.*std::log10(c);
    }
  } ;
  fft_type fftw_;
  Filter::Cascaded<frequency_vector<double> > filter_;
  std::string host_;
  std::string port_;
  double fmin_Hz_;
  double fmax_Hz_;
  size_t spec_counter_;
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
    ("fMin",    po::value<double>()->default_value(1439.9),    "fMin (kHz)")
    ("fMax",    po::value<double>()->default_value(1440.1),    "fMax (kHz)")
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
    config.put(".<xmlattr>.fMin_kHz", vm["fMin"].as<double>());
    config.put(".<xmlattr>.fMax_kHz", vm["fMax"].as<double>());

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
    return 1;
  }
  return 0;
}
