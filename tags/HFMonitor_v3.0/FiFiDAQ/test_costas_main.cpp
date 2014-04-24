// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
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
#include "iir.hpp"
#include "costas.hpp"

int main()
{
  const double fs(500.);
  const double tc(  0.02);
  const double fc( 10.);
  
  const double alpha(0.5);
  const double beta(alpha*alpha/4.);

  typedef filter::costas<2,
                         double, 
                         filter::iir_lowpass_1pole<double, double>, 
                         filter::integrator<double>,
                         filter::integrator_modulo<double> > costas_type;
  filter::iir_lowpass_1pole<double,double> loop_filter(tc,fs);
  filter::integrator<double>               integ_freq;
  filter::integrator_modulo<double>        integ_phase(2*M_PI);
  typedef costas_type::complex_type complex_type;
  costas_type p(fc,    fs,
                alpha, beta,
                loop_filter,
                integ_freq,
                integ_phase);
  // generate input and invoke pll
  const size_t n(5*1000);

  std::cout << "out=[\n";
  for (size_t i=0; i<n; ++i) {
    const double t(0.5+double(i)*2*M_PI*(fc+1)/fs);
    const complex_type s(  std::exp(complex_type(0., t)) * std::cos(double(i)*2*M_PI*1./fs)
                         + 0.5*complex_type(drand48()-0.5, drand48()-0.5));

    p.process(s);
    std::cout << s.real()  << " " << s.imag() << " "
              << p.theta() << " " << p.f1() << std::endl;
  }
  std::cout << "];\n";
}
