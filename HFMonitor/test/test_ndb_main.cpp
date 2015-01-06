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
#include <cmath>
#include <deque>
#include <iostream>
#include <fstream>
#include <iterator>
#include <numeric>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "FFT.hpp"
#include "Spectrum.hpp"
#include "logging.hpp"
#include "wave/reader.hpp"
//#include "polynomial_regression.hpp"
#include "repack_processor.hpp"
#include "run.hpp"

class test_proc :  processor::base_iq  {
public:
  typedef boost::shared_ptr<test_proc> sptr;
  typedef FFT::FFTWTransform<double> fft_type;
  typedef boost::shared_ptr<fft_type> fft_sptr;
  typedef frequency_vector<double> fv_type;
  // frequency -> deques with history
  typedef std::deque<double> hist_type;
  typedef frequency_vector<hist_type> fv_hist_type;
  typedef std::deque<ptime> ptime_hist_type;
  typedef boost::posix_time::time_duration time_duration;
  

  test_proc(const boost::property_tree::ptree& config)
    : base_iq(config)
    , max_time_interval_(boost::posix_time::seconds(config.get<double>("<xmlattr>.history_max_size_sec")))
    , f_min_Hz_(config.get<double>("<xmlattr>.fMin_Hz"))
    , f_max_Hz_(config.get<double>("<xmlattr>.fMax_Hz"))
    , fv_history_(f_min_Hz_, f_max_Hz_)
    , do_process_(true)
  {}

  void process_iq(processor::service_iq::sptr sp,
                  std::vector<std::complex<double> >::const_iterator i0,
                  std::vector<std::complex<double> >::const_iterator i1) {
    if (!do_process_) return;
    std::cout << "process_iq nS=" << std::distance(i0, i1) 
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << std::endl;

    const ssize_t length(std::distance(i0, i1));
    if (!fftw_) {
      std::cout << "FFTW create" << std::endl;
      fftw_ = fft_sptr(new fft_type(length, FFTW_BACKWARD, FFTW_ESTIMATE));
      std::cout << "FFTW created" << std::endl;
    }

    if (length != ssize_t(fftw_->size())) {
      std::cout << "FFTW resize " << length << std::endl;
      fftw_->resize(length);
      std::cout << "FFTW resized" << std::endl;
    }
    
    fftw_->transformRange(i0, i1, FFT::WindowFunction::Blackman<double>(length));
    const FFTSpectrum<fft_type> s(*fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    const double offset_ppb(sp->offset_ppb());
//     const fv_type ps(f_min_Hz_, f_max_Hz_, s, std::abs<double>, offset_ppb);
    fv_history_.fill(s, std::abs<double>, offset_ppb);
    ptime_history_.push_back(sp->approx_ptime());

    const time_duration dt(ptime_history_.back()-ptime_history_.front());
    if (dt > max_time_interval_) {
//       static_blanker(3.);

      //
      std::cout << "history size: " << dt << std::endl;
      std::ofstream ofs("data", std::ios::out);
      
      for (fv_hist_type::const_iterator i(fv_history_.begin()), iend(fv_history_.end()); i!=iend; ++i) {
        const double f(i->first);
        const hist_type& ft(i->second);
#if 1
        size_t epoch_counter(0);
        for (hist_type::const_iterator j(ft.begin()), jend(ft.end()); j!=jend; ++j, ++epoch_counter) {
          ofs << f << " " << epoch_counter << " " << *j << "\n";
        }
        ofs << "\n";
#endif
        // compute the auto-correlation
        const std::vector<double> ac(compute_autocorrelation(ft));

        const ssize_t min_index(40); // TODO make it relative to the sample time
        const double threshold_ac(0.5);
        ASSERT_THROW(min_index < ac.size());
        // search for a peak in the auto-correlation
        std::vector<double>::const_iterator imax(std::max_element(ac.begin()+min_index, ac.end()));
        std::vector<double>::const_iterator imin(std::min_element(ac.begin(),           ac.end()));
        const double ratio(ac[0]/(*imin));
        std::cout << "f=" << f << " " 
                  << ac[0] << " " << *imax << " " << *imin << " " << ratio << " "
                  << std::distance(ac.begin(), imax) << std::endl;
        if (ratio > 1.5) {
          // find the minimaxum peak
          const size_t period_guess(std::distance(ac.begin(), imax));
          size_t int_period(period_guess);
          double first_ac(0);
          for (int i(1); i<100 && period_guess/i>min_index; ++i) {
            ASSERT_THROW(period_guess/i < ac.size());

            const double current_ac(ac[period_guess/i]);
            if (i == 1)
              first_ac = current_ac;

            if (current_ac < threshold_ac) continue;
            
            if (current_ac/first_ac < 0.9 || current_ac/first_ac > 1.1) continue;

            int_period = period_guess/i;
            std::cout << " --- " << i << " " << period_guess/i << " " << current_ac << std::endl;
          }
          // period validation
          const double period(validate_period(ac, int_period, 0.5));//threshold_ac);
          if (period > 0.) { // validated
            // compute average signal
            const std::vector<double> avg_signal(compute_avg_signal(ft, f, period));
          }
        }
      }
    }

    // check if history is too long and remove old data
    pop_history();
  }

  void static_blanker(double threshold_factor) { // threshold = threshold_factor*mean
    std::vector<double> sum;
    for (fv_hist_type::const_iterator
           i(fv_history_.begin()),
           ibeg(fv_history_.begin()),
           iend(fv_history_.end()); i!=iend; ++i) {
      const hist_type& ft(i->second);
      if (i == ibeg)
        sum.resize(ft.size(), 0.0);

      std::vector<double>::iterator is(sum.begin());
      for (hist_type::const_iterator j(ft.begin()), jend(ft.end()); j!=jend; ++j, ++is)
        *is += *j;
    }

    // compute mean
    const double mean
      (std::accumulate(sum.begin(), sum.end(), 0.0) / (double(sum.size())));

    std::ofstream sum_ofs("sum.txt");
    sum_ofs << "# " << fv_history_.size() << " " << sum.size() << std::endl;
    
    for (int i=0; i<sum.size(); ++i)
      sum_ofs << sum[i] << std::endl;

    const double threshold(threshold_factor * mean);
    
    std::ofstream ofs("blanker.txt");
    // 5*mean -> outlier    
    for (fv_hist_type::iterator i(fv_history_.begin()),
           ibeg(fv_history_.begin()), iend(fv_history_.end()); i!=iend; ++i) {
      hist_type& ft(i->second);

      std::vector<double>::const_iterator is(sum.begin());
      for (hist_type::iterator j(ft.begin()+1), jend(ft.end()); j!=jend; ++j, ++is) {
        if (i == ibeg)
          ofs << *j << " "
              << (*j * (*is > threshold)) << " "
              << *is << " "
              << mean << std::endl;
        if (*is > threshold) // for now: use the last value
          *j = *(j-1);
      }
    }
  }

  std::vector<double> compute_avg_signal(const hist_type& h, double f, double period) {
    std::vector<double> avg_signal(1+int(period+0.5), 0.0);
    std::vector<double> avg_norm  (1+int(period+0.5), 0.0);

    std::cout << "compute_avg_signal: period=" << period << std::endl;
    char fn[1024];
    sprintf(fn, "sig_%.0f.txt", f);
    std::ofstream ofs(fn, std::ios::out);

    const int iquot(int(floor(h.size()/period)));
    std::cout << "TEST_ "
              << h.size() << " "
              << period << " "
              << iquot << " "
              << int(iquot*period) << std::endl;
    for (int i(0), n(int(iquot*period)); i<n; ++i) {
      const size_t j(int(std::fmod(double(i), period)));
      if (j >= avg_signal.size())
        std::cout << "TEST: "
                  << i << " " << j << " | "
                  << h.size() << " " << avg_signal.size() << std::endl;
      ASSERT_THROW(j >= 0);
      ASSERT_THROW(j <  avg_signal.size());
      ASSERT_THROW(i <  int(h.size()));
      avg_signal[j] += h[i];
      avg_norm[j]   += 1.;
    }
    for (int i(0), n(avg_signal.size()); i<n; ++i) {
      avg_signal[i] /= (avg_norm[i] != 0.0 ? avg_norm[i] : 1.0);
      ofs << avg_signal[i] << std::endl;
    }
    
    return avg_signal;
  }

  double validate_period(const std::vector<double>& ac, size_t int_period, double threshold_ac) {
    // check that all peaks are there
    // use latest ac peak for precice period estimate
    // returns -1 on failure

    double period(-1);
    // loop over all potential peaks
    for (int i=1, m(ac.size()/(1+int_period)); i<m; ++i) {
      ASSERT_THROW((int_period+1)*i < ac.size());
      std::vector<double>::const_iterator
        im(std::max_element(ac.begin()+(int_period-1)*i,
                            ac.begin()+(int_period+1)*i));
      period = (*im < threshold_ac
                ? -1
                : double(std::distance(ac.begin(), im))/double(i));      
      std::cout << "      -- validate_period: " << i << " " << *im << " " << period << std::endl;
      if (period < 0.)
        break;
    }

    return period;
  }

  std::vector<double> compute_autocorrelation(const hist_type& h) const {
    std::vector<double> ac(h.size()/2, 0.0);    
    for (size_t i(0), n(h.size()/2); i<n; ++i) {
      for (size_t j(0), n(h.size()/2); j<n; ++j) {
        ASSERT_THROW(i+j<h.size());
        ac[i] += h[j]*h[i+j];
      }
      if (i!=0 && ac[0]!=0)
        ac[i] /= ac[0];
    }
    ac[0] = 1;
    return ac;
  }

  void pop_history() {
    if (!ptime_history_.empty()) {
      std::cout << "pop_hist: history size: " << ptime_history_.back()-ptime_history_.front() << std::endl;
      while (ptime_history_.back()-ptime_history_.front() > max_time_interval_) {
        for (fv_hist_type::iterator i(fv_history_.begin()), iend(fv_history_.end()); i!=iend; ++i)
          i->second.pop_front();
        ptime_history_.pop_front();
        do_process_ = false;
      }
    }    
  }

private:
  fft_sptr        fftw_;
  time_duration   max_time_interval_;
  double          f_min_Hz_;
  double          f_max_Hz_;
  fv_hist_type    fv_history_;
  ptime_hist_type ptime_history_;
  bool            do_process_;
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_ndb");
  try {
    const boost::program_options::variables_map
      vm(process_options("config/multi_downconvert.xml", argc, argv));

    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config);

    wave::reader_iq<repack_processor<test_proc> > r(config.get_child("Test"));
    for (int i((argc == 1) ? 1 : 3); i<argc; ++i) {
      std::cout << "processing " << argv[i] << std::endl;
      r.process_file(argv[i]);
    }
    r.finish();

  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    LOG_ERROR(e.what()); 
    return 1;
  }
  return 0;
}
