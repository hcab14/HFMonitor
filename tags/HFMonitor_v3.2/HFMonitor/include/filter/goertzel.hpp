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
#ifndef _GOERTZEL_HPP_cm20121104_
#define _GOERTZEL_HPP_cm20121104_
#include <complex>

template<typename T> // T = complex<float> or compex<double> 
class goertzel {
public:
  typedef typename T::value_type value_type;
  goertzel(value_type kN=0)
    : q1_(0), q2_(0) {
    set_parameter(kN);
  }

  value_type kN() const { return kN_; }

  void reset() { q1_ = q2_ = 0; }
  void set_parameter(double kN) {
    kN_    = kN;
    const_ = 2*cos(2*M_PI*kN_);
    phase_     = std::exp(T(0, -2*M_PI*kN_));
    phase_inv_ = std::exp(T(0,  2*M_PI*kN_));
  }
  void update(T sample) {
    const T q0(sample + const_*q1_ - q2_);
    q2_ = q1_;
    q1_ = q0;
  }
  T x() const { return q1_*phase_inv_ - q2_; }

protected:
private:
  value_type kN_;        // normalized frequency 0..1
  value_type const_; 
  T          phase_;
  T          phase_inv_;
  T          q1_, q2_;   // filter state
} ;

#endif // _GOERTZEL_HPP_cm20121104_
