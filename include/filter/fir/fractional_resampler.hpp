// -*- C++ -*-

#ifndef _FRACTIONAL_RESAMPLER_HPP_cm170809_
#define _FRACTIONAL_RESAMPLER_HPP_cm170809_

#include <complex>
#include <vector>

#include <volk/volk.h>

#include "filter/fir.hpp"
#include "aligned_vector.hpp"

class fractional_resampler {
public:
  typedef std::complex<float> complex_type;

  // (p/q) fractional resampler
  // * length of the used FIR filter is q*l
  // * consumes q*num_blocks samples and
  // * produces p*num_blocks samples per call to process
  fractional_resampler(int p,
                       int q,
                       int l,
                       int num_blocks=1)
    : p_(p)
    , q_(q)
    , l_(l)
    , num_blocks_(num_blocks)
    , counter_(0)
    , history_(2*q*l_, 0)
    , b_(p*q*l_, 0)
    , out_(p*num_blocks, 0) {
    typedef filter::fir::lowpass<float> fir_type;
    fir_type fir(p*q*l_);
    fir.design(1.0f/p, 0.1f/p);
    for (int i=0; i<q*l; ++i) {
      for (int j=0; j<p; ++j) {
        b_[q*l-1-i + j*q*l] = p*fir.coeff()[i*p + j];
      }
    }
  }

  int p()  const { return p_; }
  int q()  const { return q_; }
  int ql() const { return q_*l_; }
  int num_blocks() const { return num_blocks_; }
  const aligned_vector<complex_type>& out() const { return out_; }

  const aligned_vector<complex_type>& process(aligned_vector<complex_type>::const_iterator i0,
                                              aligned_vector<complex_type>::const_iterator i1) {
    assert(std::distance(i0,i1) == q()*num_blocks());
    for (int output_offset=0; i0!=i1; i0+=q(), output_offset+=p()) {
      process(i0, i0+q(), output_offset);
    }
    return out_;
  }

protected:
  void process(aligned_vector<complex_type>::const_iterator i0,
               aligned_vector<complex_type>::const_iterator i1,
               int output_offset)
  {
    assert(std::distance(i0,i1) == q());
    for (int i=0, n=p()*q(); i<n; ++i) {
      const int i_mod_p(i%p());
      const int i_mod_q(i%q());
      if (!i_mod_p) {
        history_[counter_] = history_[counter_+ql()] = *i0++;
        counter_ = ((1+counter_) % ql());
      }
      if (!i_mod_q) {
        volk_32fc_32f_dot_prod_32fc(&out_[output_offset + i/q()],
                                    &history_[counter_],
                                    &b_[i_mod_p*ql()],
                                    ql());
      }
    }
  }
private:
  int p_;
  int q_;
  int l_;
  int num_blocks_;
  int counter_;
  aligned_vector<complex_type> history_;
  aligned_vector<float>        b_;
  aligned_vector<complex_type> out_;
} ;

#endif // _FRACTIONAL_RESAMPLER_HPP_cm170809_
