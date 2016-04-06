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
#ifndef _FIR_FILTER_HPP_cm131117_
#define _FIR_FILTER_HPP_cm131117_

#include <boost/math/special_functions/round.hpp>
#include <volk/volk.h>

#include "aligned_vector.hpp"
#include "filter/fir.hpp"

class fir_filter {
public:
  typedef std::complex<float> complex_type;
  typedef filter::fir::lowpass<float> fir_type;

  fir_filter(size_t n,
             double f,
             double x_slope=0.1)
    : n_(n)
    , f_(f)
    , shift_(0)
    , sample_counter_(n_-1)
    , b_(n, 0.)
    , phases_ (n, complex_type(1,0))
    , history_(n, complex_type(0,0)) {
    design(n, f, x_slope);
  }

  bool design(size_t n,
              double f,
              double x_slope=0.1) {
    n_ = n;
    f_ = f;
    b_.resize(n_, 0);
    phases_.resize(n_, 1);
    history_.resize(n_, 0);
    fir_type fir_design(n);
    fir_design.design(f, x_slope*f);
    std::copy(fir_design.coeff().begin(), fir_design.coeff().end(), b_.begin());    
    reset();
    return true;
  }

  void reset() {
    sample_counter_= n_-1;
    for (size_t i(0), n(history_.size()); i<n; ++i)
      history_[i] = 0;    
  }

  // shift by frequency f0 (normalized)
  // returns the true, quantized shift frequency
  double shift(double f0) { 
    long int shift(boost::math::lround(f0*n_));
    while (shift >= int(n_/2)) shift -= n_;
    while (shift < -int(n_/2)) shift += n_;
    shift_ = double(shift)/double(n_);
    if (is_shifted()) {
      const double two_pi_f0(2*M_PI*shift_);
      for (size_t i(0); i<n_; ++i)
        phases_[i] = std::exp(complex_type(0., two_pi_f0*i));
    }
    return shift_;
  }

  double get_shift() const { return shift_; }
  bool is_shifted() const { return shift_ != 0.; }

  void insert(complex_type sample) {
    ++sample_counter_;
    sample_counter_ %= phases_.size();
    for (size_t i(n_-1); i!=0; --i)
      history_[i] = history_[i-1];
    history_[0] = phases_[sample_counter_] * sample;
  }
  complex_type process() const {
    complex_type result(0);
    volk_32fc_32f_dot_prod_32fc(&result, history_, b_, n_);
    return result;
  }
  complex_type process(complex_type sample) {
    insert(sample);
    return process();
  }

protected:
private:
  size_t n_;                             // filter size
  double f_;                             // cutoff frequency (normalized)
  double shift_;
  size_t sample_counter_;
  aligned_vector<float> b_;              // fir coefficients
  aligned_vector<complex_type> phases_;  // phases for frequency shift
  aligned_vector<complex_type> history_; // filter history
} ;

#endif // _FIR_FILTER_HPP_cm131117_
