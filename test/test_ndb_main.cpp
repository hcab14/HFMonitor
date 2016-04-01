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
#include <map>
#include <numeric>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "FFT.hpp"
#include "Spectrum.hpp"
#include "logging.hpp"
#include "wave/reader.hpp"
#include "repack_processor.hpp"
#include "run.hpp"
#include "carrier_monitoring/background_estimator.hpp"

std::ostream& operator<<(std::ostream& os, const std::vector<double>& v) {
  os << "[";
  std::copy(v.begin(), v.end(), std::ostream_iterator<double>(os, " "));
  return os << "]";
}
std::ostream& operator<<(std::ostream& os, const std::map<double, double>& v) {
  os << "[ ";
  for (std::map<double,double>::const_iterator i(v.begin()), iend(v.end()); i!=iend; ++i)
    os << "(" << i->first << "," << i->second << ") ";
  return os << "]";
}

class k_means {
public:
  // period -> weight
  typedef std::map<double, double> map_type;
  typedef std::vector<double> vec_type;

  k_means(const vec_type& xm)
    : xm_(xm)
    , sse_(0) {}

  const vec_type& xm() const { return xm_; }
  double sse() const { return sse_; }

  template<typename Iterator>
  static map_type make_map(Iterator beg, Iterator end) {
    k_means::map_type map;
    for (Iterator i(beg); i!=end; ++i)
      ++map[*i];
    return map;
  }
  
  double step(const map_type& map) {
    sse_ = 0.;
    double sum_weights(0.);
    vec_type sum_w (xm_.size(), 0.);
    vec_type sum_wx(xm_.size(), 0.);
    for (map_type::const_iterator i(map.begin()), iend(map.end()); i!=iend; ++i) {
      double  min_dist2(0);
      ssize_t min_index(-1);
      for (vec_type::iterator jbeg(xm_.begin()), j(jbeg), jend(xm_.end()); j!=jend; ++j) {
        if (*j == 0.) continue; // ignore invalid centers
        const double_t dist2((i->first - *j)*(i->first - *j));
        if (min_index == -1 || dist2 < min_dist2) {
          min_index = std::distance(jbeg, j);
          min_dist2 = dist2;
        }
      }
      sum_w [min_index] += i->second;
      sum_wx[min_index] += i->second*i->first;
      sum_weights       += i->second;
      sse_              += i->second*min_dist2;
    }
    // update centers
    for (vec_type::iterator i(xm_.begin()), j(sum_wx.begin()), k(sum_w.begin()), iend(xm_.end()); i!=iend; ++i, ++j, ++k)
      *i = (*k != 0) ? (*j) / (*k) : 0.;

    sse_ = (sum_weights != 0) ? sse_/sum_weights : 0.;

    return sse_;
  }

protected:
private:
  vec_type xm_; // centers
  double   sse_;
} ;


class test_proc :  processor::base_iq  {
public:
  typedef boost::shared_ptr<test_proc> sptr;
  typedef FFT::FFTWTransform<float> fft_type;
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

  void process_iq(processor::service_iq::sptr sp, const_iterator i0, const_iterator i1) {
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
    
    fftw_->transformRange(i0, i1, FFT::WindowFunction::Blackman<float>(length));
    const FFTSpectrum<fft_type> s(*fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    const double offset_ppb(sp->offset_ppb());
//     const fv_type ps(f_min_Hz_, f_max_Hz_, s, std::abs<double>, offset_ppb);
    fv_history_.fill(s, std::abs<float>, offset_ppb);
    ptime_history_.push_back(sp->approx_ptime());

    const time_duration dt(ptime_history_.back()-ptime_history_.front());
    if (dt > max_time_interval_) {
      static_blanker(3.);

      //
      std::cout << "history size: " << dt << std::endl;
      std::ofstream ofs_data("data", std::ios::out);
      
      // compute mean spectrum
      std::vector<double> mean_spectrum(fv_history_.size(), 0.);      
      for (fv_hist_type::const_iterator ibeg(fv_history_.begin()), i(ibeg), iend(fv_history_.end()); i!=iend; ++i)
        mean_spectrum[std::distance(ibeg, i)] = mean(i->second.begin(), i->second.end());

      background_estimator bkg_est(fv_history_.size(), 2, 10);
      bkg_est.do_fit(mean_spectrum, 2.5);

      // save spectrum with fit
      std::ofstream ofs_spec("spec", std::ios::out);

      const double thr_db(1.5);

      for (fv_hist_type::const_iterator ibeg(fv_history_.begin()), i(ibeg), iend(fv_history_.end()); i!=iend; ++i) {
        const double f(i->first);
        const hist_type& ft(i->second);

        const ssize_t index(std::distance(ibeg, i));
        ofs_spec << "spec:" << f << " " << bkg_est.spec_db(index) << " " << bkg_est.spec_db_fitted(index) << "\n";

        // save all data
        size_t epoch_counter(0);
        for (hist_type::const_iterator j(ft.begin()), jend(ft.end()); j!=jend; ++j, ++epoch_counter) {
          ofs_data << f << " " << epoch_counter << " " << *j << "\n";
        }
        ofs_data << "\n";

        // analyze only channels with peaks in the spectrum
        if (bkg_est.spec_db(index) - bkg_est.spec_db_fitted(index) < thr_db)
          continue;

        // compute the auto-correlation
        const std::vector<double> ac(compute_autocorrelation(ft));

        const ssize_t min_index(40); // TODO make it relative to the sample time
        // threshold at -4 sigma
        const double threshold_ac(4./std::sqrt(double(ft.size())));
        ASSERT_THROW(min_index < ac.size());
        // search for a peak in the auto-correlation
        std::vector<double>::const_iterator imax(std::max_element(ac.begin()+min_index, ac.end()));
        std::vector<double>::const_iterator imin(std::min_element(ac.begin(),           ac.end()));
        std::cout << "f=" << f << " " 
                  << ac[0] << " " << *imax << " " << *imin << " "
                  << std::distance(ac.begin(), imax) << std::endl;
        if (*imin < -threshold_ac) {
          // find the first peak
          const ssize_t period_guess(std::distance(ac.begin(), imax));
          size_t int_period(period_guess);
          double first_ac(0);
          for (ssize_t i(1); i<100 && period_guess/i > min_index; ++i) {
            ASSERT_THROW(period_guess/i < ac.size());

            const double current_ac(ac[period_guess/i]);
            if (i == 1)
              first_ac = current_ac;

            if (current_ac < threshold_ac) continue;
            
            if (std::abs(current_ac-first_ac) > 0.2) continue;

            int_period = period_guess/i;
            std::cout << " --- " << i << " " << period_guess/i << " " << current_ac << std::endl;
          }
          // period validation
          double period(validate_period(ac, int_period, threshold_ac));
          if (period > 0.) { // validated
            std::cout << "period = " << period << std::endl;
//             double period_fft = validate_period_fft(ac, period);
//             std::cout << "period (fft) = " << period << std::endl;
            // compute average signal
            const std::vector<double> avg_signal(compute_avg_signal(ft, f, period));
            decode(avg_signal);
          }
        }
      }
    }

    // check if history is too long and remove old data
    pop_history();
  }
  
  void decode(const std::vector<double>& sig) {

    // determine the threshold iteratively
    std::vector<double>::const_iterator im(std::max_element(sig.begin(), sig.end()));

    double medians[2] = { *im/2, *im/2 }; // lower, upper median values
    double threshold(*im/2);
    for (size_t i=0; i<10; ++i) {
      std::vector<double> s0, s1;
      for (size_t j=0, n(sig.size()); j<n; ++j) {
	if (sig[j] == 0.0) continue;
	if (sig[j] < threshold) s0.push_back(sig[j]);
	if (sig[j] > threshold) s1.push_back(sig[j]);
      }
      std::cout << "decode: thresholds " << i << " "
		<< medians[0] << " " << medians[1] << " "
		<< threshold << std::endl;
      medians[0] = median(s0.begin(), s0.end());
      medians[1] = median(s1.begin(), s1.end());
      threshold = 0.5*(medians[0]+medians[1]);
    }

    std::vector<int> s;
    std::cout << "decode: ";
    for (size_t i(0), n(sig.size()); i<n; ++i) {
      if (sig[i] == 0.0) continue;
      s.push_back((sig[i] > threshold));
      std::cout << s.back();
    }
    std::cout << std::endl;
          
    // find the first 0->1 transition
    std::vector<int>::iterator itr(std::adjacent_find(s.begin(), s.end(), pred_bit_transition_01));

    size_t i(std::distance(s.begin(), itr));
    std::cout << "decode: i=" << i << s[i-1] << s[i] << std::endl;;  

    std::rotate(s.begin(), itr, s.end());
  
    std::vector<int> bit, bit_len;
    bit.push_back(1);
    bit_len.push_back(1);
    for (size_t j(1), n(s.size()); j<n; ++j) {
      if (s[j] == bit.back()) {
        ++bit_len.back();
      } else {
        bit.push_back(s[j]);
        bit_len.push_back(1);
      }
    }
    std::cout << "decode: ";
    for (size_t j=0; j<bit.size(); ++j) {
      std::cout << bit[j] << "," << bit_len[j] << " ";
    }
    std::cout << std::endl;

    // find dash and dot length
    k_means::map_type hist_bit_len(k_means::make_map(bit_len.begin(), bit_len.end()));
    std::cout << "hist_bit_len: "<< hist_bit_len << std::endl;
    
    k_means::vec_type xm;
    xm.push_back(2*1);
    xm.push_back(2*3);
    xm.push_back(2*7);
    xm.push_back(hist_bit_len.rbegin()->first);
    std::cout << "xm= " << xm << std::endl;
    
    k_means km(xm);
    for (size_t i=0; i<10; ++i) {
      const double sse(km.step(hist_bit_len));
      std::cout << "i= " << i << " sse= " << sse << " xm= " << km.xm()  << std::endl;
    }
    bool redo_kmeans(false);
    k_means::vec_type xmfit(km.xm());
    for (k_means::vec_type::iterator iv(xmfit.begin()), ivp(iv+1); ivp!=xmfit.end(); ) {
      if (std::abs((*ivp) - (*iv)) < 2.5) {
        *ivp = 0.5*((*ivp) + (*iv));
        iv = xmfit.erase(iv);
        ivp = iv+1;
        redo_kmeans = true;
      } else {
        ++iv;
        ++ivp;
      }
    }
    if (redo_kmeans) {
      km = k_means(xmfit);
      for (size_t i=0; i<10; ++i) {
        const double sse(km.step(hist_bit_len));
        std::cout << "[redo] i= " << i << " sse= " << sse << " xm= " << km.xm()  << std::endl;
      }      
    }
    // find dash and dot length
    std::vector<double> len_dit, len_dah;
    const size_t offset(std::distance(bit_len.begin(),
				      std::max_element(bit_len.begin(), bit_len.end())));
    for (size_t j(1), m(bit_len.size()); j<m; ++j) {
      const size_t km((offset+j-1)%m);
      const size_t k ((offset+j  )%m);
      if (bit_len[k] >= 2*bit_len[km] ) {
	len_dit.push_back(bit_len[km]);
	len_dah.push_back(bit_len[k]);
      }
      if (bit_len[km] >= 2*bit_len[k]) {
	len_dit.push_back(bit_len[k]);
	len_dah.push_back(bit_len[km]);
      }
    }
    const double ldit(median(len_dit));
    const double ldah(median(len_dah));
    std::cout << "decode: " << ldit << " " << ldah << std::endl;

    // check
    if (ldah/ldit < 2 || ldah/ldit > 4) // not OK
      return;
    
    std::string msg;
    for (size_t j(1), m(bit_len.size()); j<m; ++j) {
      const size_t k((offset+j)%m);
      if (bit[k] == 0) {
	if (bit_len[k] > 1.333*ldit)
	  msg.push_back(' ');
      }
      if (bit[k] == 1) {
	if (bit_len[k] < 0.5*(ldit+ldah)) {
	  msg.push_back('.');
	} else {
	  if (bit_len[k] < 1.5*(ldit+ldah))
	    msg.push_back('-');
	  else
	    msg.push_back('_');
	}
      }
    }
    std::cout << "decode: " << msg << std::endl;
  }
  
  static bool pred_bit_transition_01(int i, int j) {
    return (i==0 && j==1);
  }

  template<typename T>
  double mean(const T& v) const {
    return mean(v.begin(), v.end());
  }
  template<class Iterator>
  double mean(Iterator beg, Iterator end) const {
    return std::accumulate(beg, end, typename Iterator::value_type(0))/(beg==end ? 1. : double(std::distance(beg, end)));
  }

  template<class Iterator>
  double mean_notzero(Iterator beg, Iterator end) const {
    double sum_1(0), sum_x(0);
    for (Iterator i(beg); i!=end; ++i) {
      if (*i) {
        sum_1 += 1;
        sum_x += *i;
      }
    }
    return sum_1 != 0 ? sum_x/sum_1 : 0.;
  }

  template<typename T>
  double median(const T& v) const {
    T vc(v);
    return median(vc.begin(), vc.end());
  }
  template<class Iterator>
  double median(Iterator beg, Iterator end) const {
    if (beg == end) return 0.;
    typedef typename Iterator::value_type value_type;
    const ssize_t mid(std::distance(beg, end)/2);

    std::nth_element(beg, beg+mid, end);
    const value_type v(*(beg+mid));

    if ((std::distance(beg,end)%2) == 1)
      return v;

    std::nth_element(beg, beg+mid-1, end);
    const value_type v2(*(beg+mid-1));

    return 0.5*double(v+v2);
  }

  void static_blanker(double threshold_factor) { // threshold = threshold_factor*mean
    // compute the sum over frequencies
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
    const double mean_value(mean(sum.begin(), sum.end()));

    std::ofstream sum_ofs("sum.txt");
    sum_ofs << "# " << fv_history_.size() << " " << sum.size() << std::endl;
    
    for (size_t i(0), n(sum.size()); i<n; ++i)
      sum_ofs << sum[i] << std::endl;

    const double threshold(threshold_factor * mean_value);
    
    std::ofstream ofs("blanker.txt");
    // 5*mean -> outlier    
    for (fv_hist_type::iterator ibeg(fv_history_.begin()), i(ibeg), iend(fv_history_.end());
         i!=iend; ++i) {
      hist_type& ft(i->second);

      std::vector<double>::const_iterator is(sum.begin());
      for (hist_type::iterator j(ft.begin()+1), jend(ft.end()); j!=jend; ++j, ++is) {
        if (i == ibeg)
          ofs << *j << " "
              << (*j * (*is > threshold)) << " "
              << *is << " "
              << mean_value << std::endl;
        if (*is > threshold) 
          *j = 0;// *(j-1); // set to zero (or use the last value)
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
    for (int i=1, m(ac.size()/(2+int_period)); i<m; ++i) {
      ASSERT_THROW((int_period+2)*i < ac.size());
      std::vector<double>::const_iterator
        im(std::max_element(ac.begin()+(int_period-2)*i,
                            ac.begin()+(int_period+2)*i));
      period = (*im < threshold_ac
                ? -1
                : double(std::distance(ac.begin(), im))/double(i));      
      std::cout << "      -- validate_period: " << i << " " << *im << " " << period << " "
                << threshold_ac << std::endl;
      if (period < 0.)
        break;
    }

    return period;
  }

  double validate_period_fft(const std::vector<double>& ac, double estimated_period) const {
    // fourier analysis of auto-corrlelation

    estimated_period = floor(estimated_period);

    const size_t n_fft(100);
    // search in -5:0.1:5 bins
    double p [n_fft+1] = {0}; // periods
    double xs[n_fft+1] = {0}; // ac.*cos
    double xc[n_fft+1] = {0}; // ac.*sin
    double x [n_fft+1] = {0}; // abs(fft)
    for (ssize_t i(0), n(n_fft+1); i<n; ++i) {
      p[i] = estimated_period + 4./double(n_fft)*(i - 0.5*n_fft);
      const double fi(2*M_PI/p[i]);
      xs[i] = xc[i] = 0.0;
      for (size_t j(0), m(ac.size()); j<m; ++j) {
        xs[i] += std::cos(fi*j)*ac[j];
        xc[i] += std::sin(fi*j)*ac[j];
      }
      x[i] = xs[i]*xs[i] + xc[i]*xc[i];
      std::cout << "validate_period_fft " << p[i] << " " << x[i] << std::endl;
    }
    double* im = std::max_element(x, x+n_fft+1);
    return p[std::distance(x, im)];
  }

  std::vector<double> compute_autocorrelation(const hist_type& h) const {
    std::vector<double> ac(h.size()/2, 0.0);    
    const double mean_value(mean(h.begin(), h.end()));
    for (size_t i(0), n(h.size()/2); i<n; ++i) {
      for (size_t j(0), n(h.size()/2); j<n; ++j) {
        ASSERT_THROW(i+j<h.size());
        ac[i] += (h[j]-mean_value)*(h[i+j]-mean_value);
      }
      if (i!=0 && ac[0]!=0)
        ac[i] /= ac[0];
    }
    ac[0] = 1.;
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
  const double bit_len[] = {
    3, 3, 7, 8, 2, 3, 7, 3, 2, 8, 1, 3, 8, 2, 7, 13
  };
  k_means::map_type map(k_means::make_map(bit_len, bit_len+sizeof(bit_len)/sizeof(double)));
  std::cout << "hist: "<< map << std::endl;

  k_means::vec_type xm;
  xm.push_back(2*1);
  xm.push_back(2*3);
  xm.push_back(2*7);
  xm.push_back(map.rbegin()->first);
  std::cout << "xm= " << xm << std::endl;

  k_means km(xm);

  for (size_t i=0; i<10; ++i) {
    const double sse(km.step(map));
    std::cout << "i= " << i << " sse= " << sse << " xm= " << km.xm()  << std::endl;
  }

//   return EXIT_SUCCESS;

  LOGGER_INIT("./Log", "test_ndb");
  try {
    const boost::program_options::variables_map
      vm(process_options("config/multi_downconvert.xml", argc, argv));

    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config);

    wave::reader_iq<test_proc> r(config.get_child("Test"));
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
