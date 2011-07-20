// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _COSTAS_HPP_cm110707_
#define _COSTAS_HPP_cm110707_

#include <vector>
#include <complex>

namespace filter {
  // phase-locked loop
  template <typename T,
            class LOOP_FILTER,
            class INTEGRATOR_FREQ,
            class INTEGRATOR_PHASE>
  class costas {
  public:
    typedef T float_type;
    typedef typename std::complex<float_type> complex_type;
    typedef typename std::vector<complex_type> complex_vector_type;
    
    costas(double fc,
           double fs,
           double alpha,
           double beta,
           const LOOP_FILTER& loop_filter,
           const INTEGRATOR_FREQ& integrator_freq,
           const INTEGRATOR_PHASE& integrator_phase)
      : loop_filter_(loop_filter)
      , integrator_freq_(integrator_freq)
      , integrator_phase_(integrator_phase)
      , signal_(0)
      , err_(0)
      , f1_(2*M_PI*fc/fs)
      , fc_(fc)
      , fs_(fs)
      , alpha_(alpha)
      , beta_(beta) {}
    
    void reset() {
      signal_ = 0;
      err_ = 0;
      f1_ = 2*M_PI*fc_/fs_;
      loop_filter_.reset();
      integrator_freq_.reset();
      integrator_phase_.reset();
    }
    
    double process(complex_type s) {
      // update
      const complex_type i_phase(float_type(0), theta());
      const complex_type c(s * std::exp(-i_phase));
      signal_ = std::real(c) > 0. ? 1 : -1;
      err_ = signal_ * std::imag(c);           
      f1_ = 2*M_PI*fc_/fs_ + alpha_*err_ + integrator_freq_.process(beta_*err_);
      return integrator_phase_.process(f1_);
    }
    
    float_type theta() const { return integrator_phase_.get(); }
    float_type f1() const { return integrator_freq_.get(); }
    double signal() const { return signal_; }
    double err() const { return err_; }

    double alpha() const { return alpha_; }
    double beta() const { return beta_; }
    
    const LOOP_FILTER& loop_filter() const { return loop_filter_; }
    const INTEGRATOR_FREQ& integrator_freq() const { return integrator_freq_; }
    const INTEGRATOR_PHASE& integrator_phase() const { return integrator_phase_; }
  protected:
  private:
    LOOP_FILTER loop_filter_;
    INTEGRATOR_FREQ  integrator_freq_;
    INTEGRATOR_PHASE integrator_phase_;
    double signal_;
    double err_;
    double f1_;
    double fc_;
    double fs_;
    double alpha_;
    double beta_;
  } ;
} // namespace filter
#endif // _COSTAS_HPP_cm110707_
