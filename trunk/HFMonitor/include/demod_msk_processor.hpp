// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _DEMOD_MSK_PROCESSOR_HPP_cm130214_
#define _DEMOD_MSK_PROCESSOR_HPP_cm130214_

#include <complex>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "demod/msk.hpp"
#include "filter/fir.hpp"
#include "logging.hpp"
#include "processor.hpp"

class fir_filter {
public:
  typedef std::complex<double> complex_type;
  typedef filter::fir::lowpass<double> fir_type;

  fir_filter(size_t n, double f)
    : n_(n)
    , f_(f)
    , fir_design_(n)
    , b_(n, 0.)
    , phases_ (n, complex_type(1,0))
    , history_(n, complex_type(0,0)) {
    fir_design_.design(f, f/10);
    std::copy(fir_design_.coeff().begin(), fir_design_.coeff().end(), b_.begin());
  }

  double shift(double f0) { 
    long int shift(lround(f0*n_));
    std::cout << "shift= " << shift << std::endl;
    while (shift >= int(n_/2)) shift -= n_;
    while (shift < -int(n_/2)) shift += n_;
    f0 = double(shift)/double(n_);
    for (size_t i(0); i<n_; ++i) {
      phases_[i] = std::exp(complex_type(0., 2*M_PI*f0*i));
      std::cout << i << " " << phases_[i] << " " << b_[i] << std::endl;
    }
    return f0;
  }

  complex_type process(complex_type sample) {
    for (size_t i(n_-1); i>0; --i)
      history_[i] = history_[i-1];
    history_[0] = sample;
    complex_type result(0);
    for (size_t i(0); i<n_; ++i)
      result += history_[i] * phases_[i] * fir_design_.coeff()[i];
    return result;
  }

protected:
private:
  const size_t n_;
  const double f_;
  fir_type fir_design_;
  std::vector<double> b_;
  std::vector<complex_type> phases_; // for shift
  std::vector<complex_type> history_;
} ;

class demod_msk_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<demod_msk_processor> sptr;
  
  class result : public processor::result_base {
  public:
    typedef boost::shared_ptr<result> sptr;
    
    virtual ~result() {}
    static sptr make(std::string name,
                     ptime       t,
                     double amplitude,
                     double sn_db,
                     double phase_rad) {
      return sptr(new result(name, t, amplitude, sn_db, phase_rad));
    }

    double amplitude() const { return amplitude_; }
    double sn_db()     const { return sn_db_; }
    double phase_rad() const { return phase_rad_; }

    virtual std::string to_string() const {
      std::stringstream ss; 
      ss << result_base::to_string();
      ss << " " << amplitude() << " " << sn_db() << " " << phase_rad();
      return ss.str();
    }
    
  protected:
  private:
    result(std::string name,
           ptime t,
           double amplitude,
           double sn_db,
           double phase_rad)
      : result_base(name, t)
      , amplitude_(amplitude)
      , sn_db_(sn_db)
      , phase_rad_(phase_rad) {}
    
    const double amplitude_;
    const double sn_db_;
    const double phase_rad_;
  } ;
  
  typedef boost::posix_time::time_duration time_duration;

  demod_msk_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , fc_Hz_(config.get<double>("<xmlattr>.fc_Hz"))
    , fm_Hz_(config.get<double>("<xmlattr>.fm_Hz"))
    , dwl_Hz_(config.get<double>("<xmlattr>.dwl_Hz"))
    , period_Sec_(config.get<double>("<xmlattr>.period_Sec"))
    , min_SN_db_(config.get<double>("<xmlattr>.min_SN_db"))
    , filter_(201, 0.1)
    , phase_(0) {}
  
  ~demod_msk_processor() {}

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    std::cout << "demod_msk_processor::process_iq " << sp->stream_name()
              << " sample_rate_Hz="      << sp->sample_rate_Hz()
              << " center_frequency_Hz=" << sp->center_frequency_Hz()
              << " offset_ppb= " << sp->offset_ppb()
              << " offset_ppb_rms= " << sp->offset_ppb_rms()
              << " fc_Hz=" << fc_Hz_
              << " fm_Hz=" << fm_Hz_
              << std::endl;
    const double offset_ppb(sp->offset_ppb());
    const double offset_Hz(fc_Hz_*offset_ppb*1e-9);

    // set up a new filter
    bool is_first_call(false);
    if (not demod_msk_) {
      demod_msk_ = demod::msk::make(sp->sample_rate_Hz()*(1.-offset_ppb*1e-9),
                                    (fc_Hz_ - sp->center_frequency_Hz()) + offset_Hz,
                                    0.5*fm_Hz_*(1.-offset_ppb*1e-9),
                                    dwl_Hz_, period_Sec_);
      filter_.shift((fc_Hz_ - sp->center_frequency_Hz())/sp->sample_rate_Hz());
      is_first_call= true;
    }

    demod_msk_->update_ppb(sp->offset_ppb());

    for (const_iterator i(i0); i!=i1; ++i) {
      demod_msk_->process(filter_.process(*i));
      if (not is_first_call && demod_msk_->updated()) {
        const double amplitude_plus  (10*std::log10(std::abs(demod_msk_->gf_plus().x())));
        const double amplitude_minus (10*std::log10(std::abs(demod_msk_->gf_minus().x())));
        const double amplitude_center(10*std::log10(std::abs(demod_msk_->gf_center().x())));

        const double amplitude(0.5*(amplitude_plus+amplitude_minus));
        const double sn_db(amplitude-amplitude_center);

        const double delta_phase_rad(demod_msk_->delta_phase_rad() 
                                     - offset_Hz*2*M_PI*demod_msk_->period_sec()
                                     + 2*M_PI*demod_msk_->period_sec()*(fc_Hz_ - sp->center_frequency_Hz()));
        phase_ += delta_phase_rad;
        phase_ -= (phase_ >= M_PI)*2*M_PI;
        phase_ += (phase_ < -M_PI)*2*M_PI;
        std::cout << "XXX A+-0: " << amplitude_plus << " " << amplitude_minus << " " << amplitude_center
                  << " P: " << 0.5*(demod_msk_->pll_plus().theta() + demod_msk_->pll_minus().theta())
                  << " DeltaF: " << demod_msk_->delta_phase_rad() /2/M_PI/demod_msk_->period_sec()
                  << " " << delta_phase_rad /2/M_PI/demod_msk_->period_sec()
                  << " " << phase_
                  << " " << fc_Hz_ - sp->center_frequency_Hz()
                  << std::endl;
        const time_duration
          dt(0,0,0, std::distance(i0, i)*time_duration::ticks_per_second()/sp->sample_rate_Hz());
        sp->put_result(result::make(name_, sp->approx_ptime()+dt, amplitude, sn_db, phase_));
      }
    }
  }

protected:
private:
  const std::string name_;
  const double      fc_Hz_;     // center frequency
  const double      fm_Hz_;     // modulation frequency
  const double      dwl_Hz_;
  const double      period_Sec_;
  const double      min_SN_db_;
  demod::msk::sptr  demod_msk_;
  fir_filter        filter_;
  double            phase_;
} ;

#endif // _DEMOD_MSK_PROCESSOR_HPP_cm130214_
