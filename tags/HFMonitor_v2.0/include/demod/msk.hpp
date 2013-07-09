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

    typedef goertzel<complex_type>  goertzel_type;

    static sptr make(double fs_Hz,      // sampling frequency
                     double fc_Hz,      // center frequency
                     double fm_Hz,      // frequency of modulation
                     double dwl_Hz,     // bandwidth of PLL
                     double period_Sec, // Goertzel filter time period
                     double xi = 1./sqrt(2)) {
      return sptr(new msk(fs_Hz, fc_Hz, fm_Hz, dwl_Hz, period_Sec, xi));
    }

    void update_ppb(double ppb, double fc, double fm) {
      pll_plus_.update_ppb(ppb,  2*fc+fm);
      pll_minus_.update_ppb(ppb, 2*fc-fm);
    }

    void reset() {
      pll_plus_.reset();
      pll_minus_.reset();
    }

    bool   updated()         const { return sample_counter_ == 0; }
    double period_sec()      const { return period_/fs_Hz_; }
    double delta_phase_rad() const { return delta_phase_; }

    const goertzel_type& gf_plus()   const { return gf_plus_;   }
    const goertzel_type& gf_center() const { return gf_center_; }
    const goertzel_type& gf_minus()  const { return gf_minus_;  }

    const pll_type& pll_plus()  const { return pll_plus_;  }
    const pll_type& pll_minus() const { return pll_minus_; }

    void process(complex_type s) {
      if (sample_counter_ == 0) {
        gf_plus_.reset();
        gf_center_.reset();
        gf_minus_.reset();
      }
      const complex_type s2(s*s);
      pll_plus_.process(s2);
      pll_minus_.process(s2);
      gf_plus_.update(s2);
      gf_center_.update(s2);
      gf_minus_.update(s2);
      ++sample_counter_;
      if (sample_counter_ == period_) {
        sample_counter_= 0;
        const double carrier_phase(.25*(pll_plus().theta() + pll_minus().theta()));
        delta_phase_ = carrier_phase-last_phase_;
        while (delta_phase_ >   M_PI) delta_phase_ -= 2*M_PI;
        while (delta_phase_ <= -M_PI) delta_phase_ += 2*M_PI;
        last_phase_ = carrier_phase;
      }
    }
  protected:
  private: 
    static pll_type make_pll(double fs_Hz,
                             double fc_Hz,
                             double dwl_Hz,
                             double xi) {
      loop_filter l(xi, dwl_Hz, fs_Hz);
      integrator  i(8*M_PI);
      return pll_type(fc_Hz, fs_Hz, dwl_Hz, l, i);
    }

    msk(double fs_Hz,
        double fc_Hz,
        double fm_Hz,
        double dwl_Hz,
        double period_Sec,
        double xi)
      : fs_Hz_(fs_Hz)
      , period_(size_t(period_Sec*fs_Hz+0.5))
      , pll_plus_ (make_pll(fs_Hz, 2*fc_Hz+fm_Hz, dwl_Hz, xi))
      , pll_minus_(make_pll(fs_Hz, 2*fc_Hz-fm_Hz, dwl_Hz, xi))
      , gf_plus_  ((2*fc_Hz+fm_Hz)/fs_Hz)
      , gf_center_( 2*fc_Hz       /fs_Hz)
      , gf_minus_ ((2*fc_Hz-fm_Hz)/fs_Hz)
      , last_phase_(0)
      , delta_phase_(0)
      , sample_counter_(0) {
      // std::cout << "msk: " << 2*fc_Hz+fm_Hz << " " << 2*fc_Hz << " "<< 2*fc_Hz-fm_Hz << std::endl;
    }

    const double fs_Hz_;       //
    const size_t period_;      //

    pll_type   pll_plus_;      //
    pll_type   pll_minus_;     //

    goertzel_type gf_plus_;    //
    goertzel_type gf_center_;  //
    goertzel_type gf_minus_;   //

    double last_phase_;        //
    double delta_phase_;       //
    size_t sample_counter_;    //
  } ;

} // namespace demod
#endif //_DEMOD_MSK_HPP_cm130214_
