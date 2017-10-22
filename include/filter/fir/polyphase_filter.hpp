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
                   int decim,         //
                   int num_channels,  //
                   int num_taps,      // (num_taps % num_channels) == 0
                   float cutoff=0.9f)
    : num_blocks_(num_blocks)
    , decim_(decim)
    , num_channels_(num_channels)
    , num_taps_(num_taps)
    , num_taps_decim_(num_taps_/num_channels_)
    , counter_(0)
    , b_(num_taps_, 0)
    , x_(num_taps_decim_, 0)
    , history_(2*num_taps_, 0)
    , tmp_(num_blocks_*num_channels_, 0)
    , out_(num_blocks_*num_channels_, 0)
    , plan_(fftwf_plan_many_dft(1, &num_channels_, num_blocks_,
                                (fftwf_complex*)&tmp_[0], nullptr, 1, num_channels_,
                                (fftwf_complex*)&out_[0], nullptr, 1, num_channels_,
                                FFTW_FORWARD, FFTW_ESTIMATE))
  {
    typedef filter::fir::lowpass<float> fir_type;
    fir_type fir(num_taps);
    //    fir.design(1.0f/num_channels_, 0.1f/num_channels_);
    fir.design(cutoff/decim_, 0.1f/decim_);
    const fir_type::real_vector_type& b(fir.coeff());
    for (int i=0; i<num_channels_; ++i)
      for (int j=0; j<num_taps_decim_; ++j)
        b_[i*num_taps_decim_ + j] = b[i + (num_taps_decim_-j-1)*num_channels_];
  }

  ~polyphase_filter() {
    fftwf_destroy_plan(plan_);
  }

  int num_blocks()     const { return num_blocks_; }     //
  int decim()          const { return decim_; }          //
  int num_channels()   const { return num_channels_; }   //
  int num_taps()       const { return num_taps_; }       //
  int num_taps_decim() const { return num_taps_decim_; } //

  const aligned_vector<complex_type>& out() const { return out_; }

  const aligned_vector<complex_type>& process(aligned_vector<complex_type>::const_iterator i0,
                                              aligned_vector<complex_type>::const_iterator i1) {
    assert(std::distance(i0,i1) == num_blocks() * decim());

    for (int fft_idx=0; i0!=i1; ++i0) {
      history_[counter_] = history_[counter_+num_taps()] = *i0;
      counter_ = ((1+counter_) % num_taps());
      if (!(counter_ % decim())) {
        process(history_.begin()+counter_,
                history_.begin()+counter_+num_taps(),
                fft_idx++,
                counter_ % num_channels());
      }
    }

    fftwf_execute(plan_);
    return out_;
  }

protected:
  void process(aligned_vector<complex_type>::const_iterator i0,
               aligned_vector<complex_type>::const_iterator i1,
               int fft_idx,
               int fft_phase) {
    assert(std::distance(i0,i1) == num_taps());

    for (int i=0; i<num_channels(); ++i) {
      for (int j=0; j<num_taps_decim(); ++j) {
        x_[j] = i0[i + (num_taps_decim()-1-j)*num_channels()];
      }
      volk_32fc_32f_dot_prod_32fc(&tmp_[fft_idx*num_channels() + ((i+fft_phase) % num_channels())],
                                  &x_[0],
                                  &b_[i*num_taps_decim()],
                                  num_taps_decim());
    }
  }
private:
  int num_blocks_;     // number of blocks of length decim to be processed
  int decim_;          // decimation factor
  int num_channels_;   // number of channels
  int num_taps_;       // FIR filter taps
  int num_taps_decim_; // (= num_taps_ / num_channels_)

  int counter_;
  aligned_vector<float>        b_;         // fir coefficients
  aligned_vector<complex_type> x_;         // filter history
  aligned_vector<complex_type> history_;   // sample history
  aligned_vector<complex_type> tmp_;       // before IFFT
  aligned_vector<complex_type> out_;       // filter output
  fftwf_plan                   plan_;      // FFTW plan handle
} ;

#endif // _POLYPHASE_FILTER_HPP_cm170729_
