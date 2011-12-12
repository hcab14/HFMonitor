// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <iterator>
#include <complex>
#include <vector>

#include "FFT.hpp"

int main()
{
  typedef double FFTType;
  const size_t n(120000);
  std::vector<std::complex<FFTType> > in(n);
  const double f(102.5);

  for (unsigned u=0; u<n; ++u)
     in[u] = std::exp(std::complex<FFTType>(0.0,f*2.*M_PI*u/double(n)));

  FFT::FFTWTransform<FFTType> fft(in.size(), FFTW_FORWARD, FFTW_ESTIMATE);
  fft.transformVector(in, FFT::WindowFunction::Blackman<FFTType>());

  for (size_t u=0; u<fft.size(); ++u)
    std::cout << u << " " << fft.getInBin(u).real() << " " << fft.getInBin(u).imag() << " "
              << std::abs(fft.getBin(u)) << std::endl;
}
