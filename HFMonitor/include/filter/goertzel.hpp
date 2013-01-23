// -*- c++ -*-
// $Id$

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
    phase_ = std::exp(T(0, 2*M_PI*kN_));
  }
  void update(T sample) {
    const T q0(sample + const_*q1_ - q2_);
    q2_ = q1_;
    q1_ = q0;
  }
  T x() const { return q1_*phase_ - q2_; }

protected:
private:
  value_type kN_;       // normalized frequency 0..1
  value_type const_; 
  T          phase_;
  T          q1_, q2_;   // filter state
} ;

#endif // _GOERTZEL_HPP_cm20121104_
