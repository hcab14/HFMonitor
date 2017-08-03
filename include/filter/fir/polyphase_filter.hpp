// -*- C++ -*-

#ifndef _POLYPHASE_FILTER_HPP_cm170729_
#define _POLYPHASE_FILTER_HPP_cm170729_

#include <complex>
#include <vector>
#include <fftw3.h>

#include <volk/volk.h>

#include "filter/fir.hpp"
#include "aligned_vector.hpp"

class polyphase_filter {
public:
  typedef std::complex<float> complex_type;

  polyphase_filter(int num_blocks,    //
                   int num_channels,  // == decim
                   int num_taps)      // (num_taps % num_channels) == 0
    : num_blocks_(num_blocks)
    , num_channels_(num_channels)
    , num_taps_(num_taps)
    , num_taps_decim_(num_taps_/num_channels_)
    , counter_(num_taps_decim_-1)
    , b_(num_taps_, 0)
    , history_(2*num_taps_, 0)
    , tmp_(num_blocks_*num_channels_, 0)
    , out_(num_blocks_*num_channels_, 0)
#if 1
    , plan_(fftwf_plan_many_dft(1, &num_channels_, num_blocks_,
                                (fftwf_complex*)&tmp_[0], nullptr, 1, num_channels_,
                                (fftwf_complex*)&out_[0], nullptr, 1, num_channels_,
                                FFTW_FORWARD, FFTW_ESTIMATE))
#else
    , plan_(fftwf_plan_dft_1d(num_channels_,
                              (fftwf_complex*)&tmp_[0],
                              (fftwf_complex*)&out_[0],
                              FFTW_FORWARD,
                              FFTW_ESTIMATE))
#endif
  {
    typedef filter::fir::lowpass<float> fir_type;
    fir_type fir(num_taps);
    fir.design(1.0f/num_channels_, 0.1f/num_channels_);
    const fir_type::real_vector_type& b(fir.coeff());
    for (int i=0; i<num_channels_; ++i)
      for (int j=0; j<num_taps_decim_; ++j)
        b_[i*num_taps_decim_ + j] = b[i + (num_taps_decim_-j-1)*num_channels_];
  }

  ~polyphase_filter() {
    fftwf_destroy_plan(plan_);
  }

  int num_blocks()     const { return num_blocks_; }     //
  int num_channels()   const { return num_channels_; }   // == decim
  int num_taps()       const { return num_taps_; }       //
  int num_taps_decim() const { return num_taps_decim_; } //

  const aligned_vector<complex_type>& out() const { return out_; }
  const aligned_vector<complex_type>& tmp() const { return tmp_; }

  // process expects num_channels * num samples
  const aligned_vector<complex_type>& process(aligned_vector<complex_type>::const_iterator i0,
                                              aligned_vector<complex_type>::const_iterator i1) {

    assert(std::distance(i0,i1) == num_blocks() * num_channels());

    for (int i=0; i<num_blocks(); ++i) {
      counter_ = ((1+counter_) % num_taps_decim());

      for (int j=0; j<num_channels(); ++j, ++i0) {
        const int idx = (2*j+1)*num_taps_decim() - 1 - counter_;
        history_[idx] = history_[idx+num_taps_decim()] = *i0;
        volk_32fc_32f_dot_prod_32fc(&tmp_[i*num_channels() + j],
                                    &history_[idx],
                                    &b_[j*num_taps_decim()],
                                    num_taps_decim());
      }
    }
    // IFFT
    fftwf_execute(plan_);
    return out_;
  }

protected:
private:
  int num_blocks_;   //
  int num_channels_; // == decim
  int num_taps_;
  int num_taps_decim_;

  int counter_;

  aligned_vector<float>        b_;         // fir coefficients
  aligned_vector<complex_type> history_;   // filter history
  aligned_vector<complex_type> tmp_;       // before IFFT
  aligned_vector<complex_type> out_;       // filter output
  fftwf_plan                   plan_;      //
} ;

#endif // _POLYPHASE_FILTER_HPP_cm170729_
