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
  fft.transformVector(in, FFT::WindowFunction::Blackman<FFTType>(in.size()));

  for (size_t u=0; u<fft.size(); ++u)
    std::cout << u << " " << fft.getInBin(u).real() << " " << fft.getInBin(u).imag() << " "
              << std::abs(fft.getBin(u)) << std::endl;
}
