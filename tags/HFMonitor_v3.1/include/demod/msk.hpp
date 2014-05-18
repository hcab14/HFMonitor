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
                     double fm_Hz,      // frequency of modulation \equiv baud/2
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
      sumE_  = sumO_  = 0.;
      signE_ = signO_ = 0;
      bitE_  = bitO_  = false;
    }

    bool   phase_available() const { return sample_counter_ == 0; }
    double period_sec()      const { return period_/fs_Hz_; }
    double delta_phase_rad() const { return delta_phase_; }

    bool   bit_available() const   { return bit_availableE_ | bit_availableO_; }
    bool   current_bit()  const    { return current_bit_; }

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
      const complex_type s2(std::pow(s, 2));
      gf_plus_.update(s2);
      gf_center_.update(s2);
      gf_minus_.update(s2);
      pll_plus_.process(s2);
      pll_minus_.process(s2);
      
      // bit stream
      //  (1) 4*carrier phase
      const double theta0(pll_plus_.theta() + pll_minus_.theta());
      //  (2) demodulated signal
      const complex_type x(s * exp(-complex_type(0, 0.25*theta0)));
      //  (3a)
      const double ct(cos(0.25*(pll_plus_.theta() - pll_minus_.theta()))); // baud/4
      const int    sign_ct(ct > 0 ? 1 : -1);
      const double xE(x.real() * ct);
      signE_ = (signE_ == 0) ? sign_ct : signE_; // init
      if (signE_ == sign_ct) { // no change in sign
        sumE_ += xE;
        bit_availableE_ = false;
      } else {
        // std::cout << "_E " << sumE_ << std::endl;
        bitE_  = (sumE_ > 0.);
        sumEold_ = sumE_;
        sumE_  = 0.;
        signE_ = sign_ct;
        current_bit_   = bitE_ ^ bitO_;
        bit_availableE_ = true;
        // std::cout << "_x " << sumEold_ << " " << sumOold_ << std::endl;
      }
      //  (3b)
      const double st(sin(0.25*(pll_plus_.theta() - pll_minus_.theta()))); // baud/4
      const int    sign_st(st > 0 ? 1 : -1);      
      const double xO(x.imag() * st);
      signO_ = (signO_ == 0) ? sign_st : signO_; // init
      if (signO_ == sign_st) { // no change in sign
        sumO_ += xO;
        bit_availableO_ = false;
      } else {
        // std::cout << "_O " << sumO_ << std::endl;
        bitO_  = (sumO_ > 0.);
        sumOold_ = sumO_;
        sumO_  = 0.;
        signO_ = sign_st;
        current_bit_   = bitE_ ^ bitO_;
        bit_availableO_ = true;
        // std::cout << "_x " << sumEold_ << " " << sumOold_ << std::endl;
      }
      //  (4)
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
        double fm_Hz, // baud/2
        double dwl_Hz,
        double period_Sec,
        double xi)
      : fs_Hz_(fs_Hz)
      , period_(size_t(period_Sec*fs_Hz+0.5))
      , pll_plus_ (make_pll(fs_Hz, 2*fc_Hz+fm_Hz, dwl_Hz, xi)) //  baud/2
      , pll_minus_(make_pll(fs_Hz, 2*fc_Hz-fm_Hz, dwl_Hz, xi)) // -baud/2
      , gf_plus_  ((2*fc_Hz+fm_Hz)/fs_Hz)
      , gf_center_( 2*fc_Hz       /fs_Hz)
      , gf_minus_ ((2*fc_Hz-fm_Hz)/fs_Hz)
      , last_phase_(0)
      , delta_phase_(0)
      , sample_counter_(0)
      , sumE_(0)
      , sumEold_(0)
      , signE_(0)
      , bitE_(false)
      , bit_availableE_(false)
      , sumO_(0)
      , sumOold_(0)
      , signO_(0)
      , bitO_(false)
      , bit_availableO_(false)
      , current_bit_(false) {
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

    double sumE_;
    double sumEold_;
    int    signE_;
    bool   bitE_;
    bool   bit_availableE_;
    double sumO_;
    double sumOold_;
    int    signO_;
    bool   bitO_;
    bool   bit_availableO_;
    bool   current_bit_;
  } ;

} // namespace demod
#endif //_DEMOD_MSK_HPP_cm130214_
