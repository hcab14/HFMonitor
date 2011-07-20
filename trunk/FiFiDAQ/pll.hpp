// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _PLL_HPP_cm110527_
#define _PLL_HPP_cm110527_

#include <vector>
#include <complex>

namespace filter {
  // phase-locked loop
  template <typename T,
            class LOOP_FILTER,
            class INTEGRATOR>
  class pll {
  public:
    typedef T float_type;
    typedef typename std::complex<float_type> complex_type;
    typedef typename std::vector<complex_type> complex_vector_type;
    
    pll(double fc,
        double fs,
        const LOOP_FILTER& loop_filter,
        const INTEGRATOR& integrator)
      : loop_filter_(loop_filter)
      , integrator_(integrator)
      , fc_(fc)
      , ts_(1./fs) {}
    
    void reset() {
      f1_= 2*M_PI*fc_;
      loop_filter_.reset();
      integrator_.reset();
    }
    
    double process(complex_type s) {
      // update
      const complex_type i_phase(float_type(0), integrator_.process(f1_*ts_));
      f1_ = 2*M_PI*fc_ + loop_filter_.process(std::arg(s * std::exp(-i_phase)));
      return theta();
    }
    
    float_type theta() const { return integrator_.get(); }
    float_type uf() const { return loop_filter_.get(); }
    float_type f1() const { return f1_; }
    
    const LOOP_FILTER& loop_filter() const { return loop_filter_; }
    const INTEGRATOR& integrator() const { return integrator_; }
  protected:
  private:
    LOOP_FILTER loop_filter_;
    INTEGRATOR  integrator_;
    double f1_;               // current nco frequency
    const double fc_;
    const double ts_;
  } ;
} // namespace filter
#endif // _PLL_HPP_cm110527_
