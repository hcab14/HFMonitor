// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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
//#undef USE_CUDA

#ifdef __APPLE__
#  include <sys/time.h>
#endif
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <iterator>
#include <complex>
#include <vector>

#include "FFT.hpp"

struct delta_time {
  delta_time(std::string msg="elapsed time: ")
    : msg_(msg) {
    gettimeofday(&start, NULL);
  }
  ~delta_time() {
    gettimeofday(&end, NULL);
    std::cout << msg_ << (end.tv_usec-start.tv_usec+1e6*(end.tv_sec-start.tv_sec))/1e6 << std::endl;
  }
  std::string msg_;
  struct timeval end, start;
} ;


#define FFT_SIZE 500000

#if 1
void test_fftw() {
  typedef double FFTType;
  const size_t n(FFT_SIZE);
  std::vector<std::complex<FFTType> > in(n);
  { delta_time dt("init: ");
    const double f(102.5);
    for (unsigned u=0; u<n; ++u)
      in[u] = std::complex<FFTType>(0.1)+std::exp(std::complex<FFTType>(0.0,f*2.*M_PI*u/double(n)));
  }
  FFT::FFTWTransform<FFTType> fft(in.size(), FFTW_FORWARD, FFTW_ESTIMATE);
  for (int i(0); i<10; ++i ) {
    delta_time dt("FFTW: ");
    fft.transformVector(in, FFT::WindowFunction::Rectangular<FFTType>(in.size()));
  }
  for (size_t i(100); i<110; ++i)
    std::cout << fft.getBin(i) << std::endl;

}
#endif

void test_cufft(int argc, char* argv[])
{
#ifdef USE_CUDA
  findCudaDevice(argc, (const char **)argv);
  std::vector<std::complex<float> > v(FFT_SIZE);

  { delta_time dt("init: ");
    const double f(102.5);
    for (unsigned u=0, n=FFT_SIZE; u<n; ++u) {
      const std::complex<float> s(std::complex<float>(0.1)+std::exp(std::complex<float>(0.0,f*2.*M_PI*u/double(n))));
      v[u] = s;
    }
  }
  
  FFT::CUFFTTransform cufft(FFT_SIZE, CUFFT_FORWARD, 0);
  for (int i(0); i<10; ++i ) { delta_time dt("CUFFT: ");
    cufft.transformVector(v, FFT::WindowFunction::Rectangular<float>(FFT_SIZE));
  } 
  for (size_t i(100); i<110; ++i) {
    std::complex<float> cc(cufft.out(i));
    cc /= FFT_SIZE;
    std::cout << cufft.getBin(i) << " " << cc<< std::endl;
  }

#endif
}

int main(int argc, char* argv[])
{
  {delta_time dt("test_fftw: ");
    test_fftw();
  }
  {delta_time dt("test_cuda: ");
  test_cufft(argc, argv);
  }
}
