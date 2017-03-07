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
#include <iostream>
#include <fstream>

#include "filter/fir.hpp"
#include "filter/fir/overlap_save.hpp"

int main()
{
  typedef float float_type;
  typedef filter::fir::lowpass<float_type> fir_type;
  typedef filter::fir::overlap_save os_type;
  const size_t m(125001);
  fir_type fir(m);
  fir.design(0.009, 0.009/5);

#if 1
  const size_t l(500000);

  os_type os(l, m);
  size_t decim(10);
  std::pair<size_t, float> r(os.add_filter(fir.coeff(), 0.1, decim));
  std::cout << "#offset = " << r.second << " (" << 0.1 << ")" << std::endl;
  os_type::complex_vector_type buffer(l);


  FFT::FFTWTransform<float> t1(l,       FFTW_FORWARD, FFTW_ESTIMATE);
  FFT::FFTWTransform<float> t2(l/decim, FFTW_FORWARD, FFTW_ESTIMATE);

  size_t counter(0);
  for (size_t i=0; i<5; ++i) {
    for (size_t j=0; j<l; ++j, ++counter) {
      buffer[j]  = float(0)*os_type::complex_type(drand48()-0.5, drand48()-0.5);
      buffer[j] += std::exp(os_type::complex_type(0., 2*M_PI*0.10*counter));
    }
    os.proc(buffer);


    if (i>1) {
      const os_type::complex_vector_type out(os.begin(r.first),
                                             os.end(r.first));
      t1.transformVector(buffer, FFT::WindowFunction::Blackman<float>(l));
      t2.transformVector(out, FFT::WindowFunction::Blackman<float>(l/decim));
      for (size_t j=0; j<l; ++j) {
        std::cout << "O " << j << " "
                  << std::scientific << " "
                  << std::abs(t1.getBin(j)) << std::endl;;
      }
      for (size_t j=0; j<l/decim; ++j) {
        std::cout << "D " << j*decim << " "
                  << std::scientific << std::abs(t2.getBin(j)) << std::endl;;
      }
    }
  }
#endif
  // typedef os_type ols;
  // std::cerr << "do " << 100 << " " << ols::design_optimal(100) << std::endl;
  // std::cerr << "do " << 200 << " " << ols::design_optimal(200) << std::endl;
  // std::cerr << "do " << 500 << " " << ols::design_optimal(500) << std::endl;
  // std::cerr << "do " << 1000 << " " << ols::design_optimal(1000) << std::endl;
  // std::cerr << "do " << 2000 << " " << ols::design_optimal(2000) << std::endl;
  // std::cerr << "do " << 5000 << " " << ols::design_optimal(5000) << std::endl;
  // std::cerr << "do " << 10000 << " " << ols::design_optimal(10000) << std::endl;
  // std::cerr << "do " << 50000 << " " << ols::design_optimal(50000) << std::endl;
}
