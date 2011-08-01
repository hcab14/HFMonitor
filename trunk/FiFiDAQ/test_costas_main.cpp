// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
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
