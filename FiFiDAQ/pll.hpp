// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _PLL_HPP_cm110527_
#define _PLL_HPP_cm110527_

#include <vector>
#include <complex>

#include "IIR.hpp"

// 2nd order phase-locked loop
template <typename T>
class pll {
public:
    typedef T float_type;
    typedef typename std::complex<float_type> complex_type;
    typedef typename std::vector<complex_type> complex_vector_type;

  pll()
    : fc_(0.)
    , ts_(1.) {
    a_[0] = 1.;
    a_[1] = -1.;
    b_[0] = 0.;
    b_[1] = 0.;
  }

  pll(double xi,
      double dwl,
      double fc,
      double fs) {
    init(xi, dwl, fc, fs);
  }

  void init(double xi,
            double dwl,
            double fc,
            double fs) {
    const double wl = dwl*2*M_PI;
    const double wn = wl/2/xi;
    const double k0 = 1.;
    const double kd = 1.;
    const double tau1 = k0*kd/wn/wn;
    const double tau2 = xi*2/wn;
    ts_ = 1./fs;
    // loop filter coefficients from s -> z transform with prewarping
    //   (1 + s*tau1) / (s*tau2)
    a_[0] =  1.0;
    a_[1] = -1.0;
    b_[0] = ts_/2/tau1*(1.+1./std::tan(ts_/2/tau2));
    b_[1] = ts_/2/tau1*(1.-1./std::tan(ts_/2/tau2));
    fc_ = fc;
    iir_.init(std::vector<double>(a_, a_+2),
              std::vector<double>(b_, b_+2));
    reset();
  }

  void reset() {
    theta_[0] = theta_[1] = 0.0;
    uf_[0] = uf_[1] = 0.0;
    ud_[0] = ud_[1] = 0.0;
    f1_[0] = f1_[1] = 2*M_PI*fc_;
  }

  void process(complex_type s) {
    // update
    theta_[1] = theta_[0] + f1_[0]*ts_;
    ud_[1] = std::arg(s * std::exp(complex_type(float_type(0), -theta_[1])));
    uf_[1] = -a_[1]*uf_[0] + b_[0]*ud_[1] + b_[1]*ud_[0];
    iir_.process(ud_[1]);
    f1_[1] = 2*M_PI*fc_ + uf_[1];

    // old <- new
    theta_[0] = theta_[1];
    ud_[0] = ud_[1];
    uf_[0] = uf_[1];
    f1_[0] = f1_[1];
  }

  float_type theta() const { return theta_[1]; }
  float_type ud() const { return ud_[1]; }
  float_type uf__() const { return uf_[1]; }
  float_type uf() const { return iir_.get(); }
  float_type f1() const { return f1_[1]; }
protected:

private:
  double fc_;
  double ts_;
  // loop filter coefficients
  double a_[2];
  double b_[2];

  iir<double, double, 2> iir_;

  // state
  double theta_[2]; // phase
  double ud_[2];    // used for loop filter
  double uf_[2];    // used for loop filter
  double f1_[2];    // nco frequency
} ;

#endif // _PLL_HPP_cm110527_
