// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _DEMOD_MSK_HPP_cm130214_
#define _DEMOD_MSK_HPP_cm130214_

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include "filter/iir.hpp"
#include "filter/pll.hpp"
#include "filter/goertzel.hpp"

namespace demod {  
  class msk : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<msk> sptr;

    typedef filter::loop_filter_2nd<double> loop_filter;
    typedef filter::integrator_modulo<double> integrator;
    typedef filter::pll<double, loop_filter, integrator> pll_type;
    typedef pll_type::complex_type complex_type;

    typedef goertzel<complex_type>  goertzel_t;

    static sptr make(double fs_Hz,  // sampling frequency
                     double fc_Hz,  // frequency of modulation
                     double dwl_Hz, // "bandwidth" of PLL
                     double period_Sec,
                     double xi  = 1./sqrt(2)) {
      return sptr(new msk(fs_Hz, fc_Hz, dwl_Hz, period_Sec, xi));
    }

    void process(complex_type s) {
      const complex_type s2(s*s);
      pll_plus_.process(s2);
      pll_minus_.process(s2);

      ++sample_counter_;

      if (sample_counter_ == period_) {
        sample_counter_= 0;

        double amplitude_plus   = std::abs(gf_plus_.x());
        double amplitude_center = std::abs(gf_center_.x());
        double amplitude_minus  = std::abs(gf_minus_.x());
        
        const double carrier_phase(.5*(pll_plus().theta() + pll_minus().theta()));
        double delta_phase = carrier_phase-last_phase_;
        while (delta_phase >   M_PI) delta_phase -= 2*M_PI;
        while (delta_phase <= -M_PI) delta_phase += 2*M_PI;

        std::cout << "XXX A+-0: " << amplitude_plus << " " << amplitude_minus << " " << amplitude_center
                  << " P: " << carrier_phase
                  << " DeltaF: " << 0.5*delta_phase /2/M_PI*fs_Hz_/period_
                  << std::endl;
        
        last_phase_ = carrier_phase;
        
        gf_plus_.reset();
        gf_center_.reset();
        gf_minus_.reset();
      }
      gf_plus_.update(s2);
      gf_center_.update(s2);
      gf_minus_.update(s2);
    }

    const pll_type& pll_plus()  const { return pll_plus_; }
    const pll_type& pll_minus() const { return pll_minus_; }

  protected:
  private: 
    static pll_type make_pll(double fs_Hz,
                             double fc_Hz,
                             double dwl_Hz,
                             double xi) {
      loop_filter l(xi, dwl_Hz, fc_Hz, fs_Hz);
      integrator  i(4*M_PI);
      return pll_type(fc_Hz, fs_Hz, l, i);
    }

    msk(double fs_Hz,
        double fc_Hz,
        double dwl_Hz,
        double period_Sec,
        double xi)
      : fs_Hz_(fs_Hz)
      , period_(size_t(period_Sec*fs_Hz+0.5))
      , pll_plus_ (make_pll(fs_Hz,  fc_Hz, dwl_Hz, xi))
      , pll_minus_(make_pll(fs_Hz, -fc_Hz, dwl_Hz, xi))
      , gf_plus_(fc_Hz/fs_Hz)
      , gf_center_(0)
      , gf_minus_(-fc_Hz/fs_Hz)
      , last_phase_(0)
      , sample_counter_(0) {}
    
    const double fs_Hz_;
    const size_t period_;

    pll_type   pll_plus_;
    pll_type   pll_minus_;

    goertzel_t gf_plus_;
    goertzel_t gf_center_;
    goertzel_t gf_minus_;

    double     last_phase_;
    size_t     sample_counter_;

  } ;

} // namespace demod
#endif //_DEMOD_MSK_HPP_cm130214_
