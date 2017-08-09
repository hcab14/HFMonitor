// -*- C++ -*-
// $Id$

#include <iostream>
#include <cstdlib>

#include "filter/fir/fractional_resampler.hpp"

int main(int argc, char* argv[])
{
  fractional_resampler fresamp(19, 20, 5);
  aligned_vector<std::complex<float> > x(10000, 0);
  for (int i=0, n=x.size(); i<n; ++i) {
    x[i]  =  0.01f*std::complex<float>(drand48()-0.5, drand48()-0.5);
    x[i] += 10.0f*std::exp(std::complex<float>(0., 2*M_PI*0.02*i));
    x[i] += 10.0f*std::exp(std::complex<float>(0., 2*M_PI*0.22*i));
    x[i] += 10.0f*std::exp(std::complex<float>(0., 2*M_PI*0.42*i));
    x[i] += 10.0f*std::exp(std::complex<float>(0., 2*M_PI*0.62*i));
    x[i] += 10.0f*std::exp(std::complex<float>(0., 2*M_PI*0.82*i));
    x[i] += 10.0f*std::exp(std::complex<float>(0., 2*M_PI*0.92*i));
    std::cout << "X " << x[i].real() << " " << x[i].imag() << std::endl;
    if (!(i%fresamp.q())) {
      fresamp.process(x.begin()+i-fresamp.q(), x.begin()+i);
      const aligned_vector<std::complex<float> > &y = fresamp.out();
      for (auto s : y)
        std::cout << "Y " << s.real() << " " << s.imag() << std::endl;
    }
  }
  return EXIT_SUCCESS;
}
