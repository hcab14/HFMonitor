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
#include <deque>
#include <iostream>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "FFT.hpp"
#include "Spectrum.hpp"
#include "wave/reader.hpp"
#include "polynomial_regression.hpp"
#include "run.hpp"

class fft_calibration : public processor::base_iq {
public:
  typedef boost::shared_ptr<fft_calibration> sptr;
  typedef FFT::FFTWTransform<double> fft_type;
  typedef frequency_vector<double> fv_type;
  // frequency -> deques with history
  typedef frequency_vector<std::deque<double> > fv_phase_hist_type;
  typedef std::deque<int> fmax_index_hist_type;
  typedef boost::shared_ptr<polynomial_interval_fit> pif_sptr;
  typedef boost::posix_time::time_duration time_duration;

  typedef enum {
    STATE_INIT  = 0,
    STATE_OK    = 1,
    NUM_STATES
  } state_t;

  static std::string state2str(state_t s) {
    static std::string stab[] = { "INIT", "OK" };
    assert(s < NUM_STATES && s >= 0);
    return stab[s];
  }

  state_t state() const { return state_; }

  static sptr make(const boost::property_tree::ptree& config, double shift) {
    return sptr(new fft_calibration(config, shift));
  }

  virtual ~fft_calibration() {}

  double rms() const {
    if (!pif_ || f_history_.size() == 0) return 1e10;
    return std::sqrt(pif_->chi2()/f_history_.size());
  }
  std::pair<double, double> eval(ptime t) const {
    const time_duration dt(t-ptime_of_first_measurement_);
    const double dt_sec(1e-3*double(dt.total_milliseconds()));
    const double tt(dt_sec/delta_t_sec_);
    // std::cout << "eval: " << str(boost::format("%+5.3f ") % shift_) << t 
    //           << " last measurement: " << ptime_of_first_measurement_
    //           << " dt: " << dt
    //           << " dt_sec: " << dt_sec
    //           << " tt: " << tt
    //           << std::endl;
    return eval(tt);
  }
  std::pair<double, double> eval(double t) const {
    if (!pif_) return std::make_pair(0., 1e10);
    std::pair<double, double> r(pif_->eval(t));
    r.second *= rms();
    return r;
  }

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    const ssize_t length(std::distance(i0, i1));
#if 1
    std::cout << "process_iq nS=" << std::distance(i0, i1)
              << " " << sp->id()
              << " " << sp->approx_ptime()
              << " " << sp->sample_rate_Hz()
              << " " << sp->center_frequency_Hz()
              << " " << sp->offset_ppb()
              << " " << sp->offset_ppb_rms()
              << " length=" << length
              << std::endl;
#endif
    if (length != ssize_t(fftw_.size()))
      fftw_.resize(length);
    f_shift_Hz_  = shift_*double(sp->sample_rate_Hz())/double(length);
    delta_t_sec_ = length/sp->sample_rate_Hz();

    // f_shift_Hz_ += 0.0116+1./6.;
    // const double df_shift = -0.0001;
    std::vector<std::complex<double> > sv(length, 0);
    for (ssize_t i(0); i<length; ++i) {
      const double t(sample_counter_++/double(sp->sample_rate_Hz()));
      sv[i] = *(i0+i) * std::exp(std::complex<double>(0,2*M_PI*f_shift_Hz_*t));
    }
    // sample_counter_ %= 3*length;
    fftw_.transformRange(sv.begin(), sv.end(), FFT::WindowFunction::Blackman<double>(length));
    const FFTSpectrum<fft_type> s(fftw_, sp->sample_rate_Hz(), sp->center_frequency_Hz());
    const double offset_ppb(0); // to be determined
    const fv_type ps(f_min_Hz_, f_max_Hz_, s, std::abs<double>, offset_ppb);
    phase_history_.fill(s, std::arg<double>, offset_ppb);

    if (fmax_index_history_.empty()) {
      // const time_duration dt(0,0,0, int64_t(0.5 + time_duration::ticks_per_second()*0.5*delta_t_sec_));
      ptime_of_first_measurement_ = sp->approx_ptime(); // time of first phase measurement
    }

    const fv_type::const_iterator 
      i_max(std::max_element(ps.begin(), ps.end(), fv_type::cmpSecond));
    fmax_index_history_.push_back(std::distance(ps.begin(), i_max));

    std::cout << str(boost::format("max: %+5.3f %f %f phase= %f")
                     % shift_
                     % i_max->first
                     % i_max->second
                     % phase_history_[std::distance(ps.begin(), i_max)].second.back()
                     )
              << std::endl;
    while (fmax_index_history_.size() > phase_history_max_size_)
      pop_front();

    state_ = compute_df(sp->center_frequency_Hz());
    if (state_ == STATE_OK) { // perform fit
      std::vector<double> index_vector(2,0);
      index_vector[0] = 0;
      index_vector[1] = f_history_.size();      
      std::vector<size_t> b(f_history_.size(), 1);
      pif_ = pif_sptr(new polynomial_interval_fit(2, index_vector));

      std::vector<double> t(f_history_.size(), 0);
      for (size_t i(0), n(t.size()); i<n; ++i)
        t[i] = i+0.5; // times when measurements (phase differences) were taken 

      double scale(0);
      size_t n_changed(1);
      for (size_t counter(0); counter<100 && n_changed!=0; ++counter) {
        if (false == pif_->fit(t, f_history_, b)) {
          clear();
          state_ = STATE_INIT;
          return;
        }
        scale = std::sqrt(pif_->chi2() / f_history_.size());
        n_changed = 0;
        for (size_t i(0); i<f_history_.size(); ++i) {
          const std::pair<double, double> ff(pif_->eval(i));
          const size_t accepted(std::abs(f_history_[i]-ff.first) < 15*scale*std::sqrt(ff.second) ? 1 : 0);
          // const size_t accepted(std::abs(f_history_[i]-ff.first) < 30*1e-9 ? 1 : 0);
          n_changed += (b[i] != accepted);
          b[i] = accepted;
          // std::cout << str(boost::format("fit: %+5.3f %3d %3d %.3e %.3e +- %.3e %d\n")
          //                  % shift_
          //                  % counter % i
          //                  % f_history_[i]
          //                  % ff.first
          //                  % (15*scale*std::sqrt(ff.second))
          //                  % b[i]
          //                  );
        }
        const std::pair<double, double> ff(pif_->eval(f_history_.size()));
        // std::cout << str(boost::format("fit: %+5.3f %3d (%.5e+-%.5e) (%.5e+-%.5e) (%.5e+-%.5e) chi2/dof = %.2e/%d (%.5e+-%.5e)\n" )
        //                  % shift_
        //                  % counter
        //                  % (1e9*pif_->x()(0)) % (1e9*scale/std::sqrt(pif_->q()(0,0)))
        //                  % (1e9*pif_->x()(1)) % (1e9*scale/std::sqrt(pif_->q()(1,1)))
        //                  % (1e9*pif_->x()(2)) % (1e9*scale/std::sqrt(pif_->q()(2,2)))
        //                  % pif_->chi2()
        //                  % pif_->dof()
        //                  % ff.first
        //                  % (scale*std::sqrt(ff.second))
        //                  );
      }
    }
  }

protected:
  void pop_front() {
    for (fv_phase_hist_type::iterator i(phase_history_.begin()), end(phase_history_.end());
         i!=end; ++i)
      i->second.pop_front();
    fmax_index_history_.pop_front();
    const time_duration dt(0,0,0, int64_t(0.5 + time_duration::ticks_per_second()*delta_t_sec_));
    ptime_of_first_measurement_ += dt;

  }
  void clear() {
    for (fv_phase_hist_type::iterator i(phase_history_.begin()), end(phase_history_.end());
         i!=end; ++i)
      i->second.clear();
    fmax_index_history_.clear();
  }
  state_t compute_df(double f0) {
    // (1) accumulating measurements
    if (fmax_index_history_.size() < phase_history_max_size_/2)
      return STATE_INIT;
    // (2) check peak positions and compute df
    const double max_delta_phase(0.9*M_PI); // maximum allowed phase variation
    f_history_.clear();
    f_history_.resize(fmax_index_history_.size() - 1, 0.);
    for (fmax_index_hist_type::iterator i(fmax_index_history_.begin()), j(i+1), 
           end(fmax_index_history_.end()); j!=end; ++i, ++j) {
      const int abs_delta_ij(std::abs(*i-*j));
      const ssize_t k(std::distance(fmax_index_history_.begin(), i));
      if (abs_delta_ij > 0) { // distance between maxima is more than one bin
        goto fail;
      }
      if (abs_delta_ij == 0) { // maxima in the same FFT bins
        const double delta_phase(diff_unwrapped(phase_history_[*i].second[k],
                                                phase_history_[*i].second[k+1]));
        if (std::abs(delta_phase) > max_delta_phase)
          goto fail;
        f_history_[k] = (phase_history_[*i].first
                         - delta_phase/delta_t_sec_/(2*M_PI)
                         + f_shift_Hz_ - f0)/f0;
        // std::cout << str(boost::format("A shift=%+5.3f ff=%f delta_phase=%f\n")
        //                  % shift_
        //                  % (f0*(1.+f_history_[k]))
        //                  % delta_phase);
      }
    }
    return STATE_OK;
  fail:
    clear();
    return STATE_INIT;
  }
private:
  fft_calibration(const boost::property_tree::ptree& config, double shift)
    : base_iq(config)
    , fftw_(1024, FFTW_BACKWARD, FFTW_ESTIMATE)
    , phase_history_max_size_(config.get<double>("<xmlattr>.phase_history_max_size"))
    , f_min_Hz_(config.get<double>("<xmlattr>.fMin_Hz"))
    , f_max_Hz_(config.get<double>("<xmlattr>.fMax_Hz"))
    , shift_(shift)
    , f_shift_Hz_(0)
    , phase_history_(f_min_Hz_, f_max_Hz_)
    , ptime_of_first_measurement_()
    , delta_t_sec_(1.)
    , state_(STATE_INIT)
    , sample_counter_(0) {
  }
  
  double diff_unwrapped(double p1, double p2) {
    double dp(p2-p1);
    while (dp > M_PI)
      dp -= 2*M_PI;
    while (dp <= -M_PI)
      dp += 2*M_PI;
    return dp;
  }
  fft_type             fftw_;
  size_t               phase_history_max_size_;
  double               f_min_Hz_;
  double               f_max_Hz_;
  double               shift_;       // shift [normalized frequency]
  double               f_shift_Hz_;  // shift [Hz]
  fv_phase_hist_type   phase_history_;
  fmax_index_hist_type fmax_index_history_;
  ptime                ptime_of_first_measurement_;
  double               delta_t_sec_;
  state_t              state_;
  int64_t              sample_counter_;
  std::vector<double>  f_history_;
  pif_sptr             pif_;
} ;

/*! \addtogroup processors
 *  @{
 * \addtogroup cal cal
 * new calibration processor
 * 
 * @{
 */

/// new calibration processor
class calibration_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<calibration_processor> sptr;

  /// result of @ref calibration_processor
  class result : public processor::result_base {
  public:
    typedef boost::shared_ptr<result> sptr;

    virtual ~result() {}
    static sptr make(std::string name,
                     ptime       t) {
      return sptr(new result(name, t));
    }
    

    virtual std::ostream& dump_header(std::ostream& os) const {
      return os;
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      return os;
    }

  protected:
  private:
    result(std::string name,
           ptime t)
      : result_base(name, t) {}
  } ;

  typedef boost::posix_time::time_duration time_duration;

  calibration_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , n_filter_(config.get<int>("<xmlattr>.n_filter")) {
    ASSERT_THROW(n_filter_>0 && (n_filter_%2) == 1);
    const int m((n_filter_-1)/2);
    for (int i(-m); i<=m; ++i)
      fft_cal_.push_back(fft_calibration::make(config, double(i)/double(n_filter_)));
  }
  
  virtual ~calibration_processor() {}

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    std::vector<double> rms(n_filter_, 0);
    size_t n_ok(0);
    for (ssize_t i(0); i<n_filter_; ++i) {
      fft_cal_[i]->process_iq(sp, i0, i1);

      const bool is_ok(fft_cal_[i]->state() == fft_calibration::STATE_OK);
      n_ok += is_ok;
      rms[i] = (is_ok ? fft_cal_[i]->rms() : 1e10);
    }
    if (n_ok < 2) {
      // error
    }
    std::vector<double>::iterator 
      rms_min(std::min_element(rms.begin(), rms.end()));
    const ssize_t i_min(std::distance(rms.begin(), rms_min));

    // consistency test
    const std::pair<double, double> fi_min(fft_cal_[i_min]->eval(sp->approx_ptime()));
    size_t tests_passed(0);
    for (ssize_t i(0); i<n_filter_; ++i) {
      if (i == i_min) continue;
      if (fft_cal_[i]->state() != fft_calibration::STATE_OK) continue;
      const std::pair<double, double> fi(fft_cal_[i]->eval(sp->approx_ptime()));
      const double d_nsigma(std::abs(fi.first-fi_min.first)
                            /std::sqrt(fi.second*fi.second + fi_min.second*fi_min.second));
      std::cout << str(boost::format("consistency test: %2d %2d (%.5e+-%.5e) (%.5e+-%.5e) %.2f\n")
                       % i_min
                       % i
                       % fi_min.first
                       % fi_min.second
                       % fi.first
                       % fi.second
                       % d_nsigma
                       );
      tests_passed += (d_nsigma < 3.);  // max allowed difference: 3 sigma 
    }
    if (tests_passed == n_ok-1) {
      // TODO
    }
  }

protected:  
  int n_filter_; // number of filters: [-m:m]/n with m=(n-1)/2
  std::vector<fft_calibration::sptr> fft_cal_;
} ;
/// @}
/// @}


int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_tgf");
  try {
    const boost::program_options::variables_map
      vm(process_options("config/multi_downconvert.xml", argc, argv));

    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config);
    config.get_child("Calibration");

    wave::reader_iq<calibration_processor> r(config.get_child("Calibration"));
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
