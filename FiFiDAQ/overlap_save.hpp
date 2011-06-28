// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _OVERLAP_SAVE_HPP_cm110628_
#define _OVERLAP_SAVE_HPP_cm110628_

#include <iostream>
#include "FFT.hpp"

template<typename T>
class overlap_save {
public:  
  typedef std::complex<T> complex_type;
  typedef std::vector<T> real_vector_type;
  typedef std::vector<complex_type> complex_vector_type;
  typedef FFT::FFTWTransform<T> fft_type;

  overlap_save(size_t l, //
	       size_t m) //
    : l_(l)
    , m_(m)
    , fft_ (l+m-1,       1, FFTW_ESTIMATE)
    , ifft_(fft_.out(), -1, FFTW_ESTIMATE)
    , h_(l+m-1,0) {}

  template<typename U>
  void update_filter_coeff(const typename std::vector<U>& b) {
    if (b.size() != m_)
      throw std::runtime_error("overlap_save::update_filter_coeff b.size() != m_");
    complex_vector_type in(l_+m_-1, 0);
    std::copy(b.begin(), b.end(), in.begin());
    fft_.transformVector(in, FFT::WindowFunction::Rectangular<T>());
    for (size_t i=0; i<l_+m_-1; ++i) {
      h_[i] = fft_.out(i);
      fft_.in(i) = 0;
    }
  }

  complex_vector_type proc(const complex_vector_type& in) {
    if (in.size() != l_)
      throw std::runtime_error("overlap_save::proc in.size() != l_");
    // copy input data
    for (size_t i=0; i<l_; ++i)
      fft_.in(m_+i-1) = in[i];
    fft_.transform();
    // multiply with fourier transformed filter coeff
    for (size_t i=0; i<l_+m_-1; ++i)
      ifft_.in(i) *= h_[i];
    ifft_.transform();
    // save old samples
    for (size_t i=0; i<m_-1; ++i)
      fft_.in(i) = fft_.in(l_+i);
    // return result, omitting first m_-1 samples
    complex_vector_type result(in);
    for (size_t i=0; i<l_; ++i)
      result[i] = ifft_.out(m_-1+i);
    return result;
  }

private:
  const size_t l_;
  const size_t m_;
  fft_type fft_;
  fft_type ifft_;
  complex_vector_type h_;
} ;

#endif // _OVERLAP_SAVE_HPP_cm110628_
