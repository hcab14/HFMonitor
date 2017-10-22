// -*- C++ -*-
// $Id$

#include <iostream>
#include <cstdlib>

#include "filter/fir/fractional_resampler.hpp"
#include "filter/fir.hpp"

int main(int argc, char* argv[])
{
#if 1
#if 0
  const int   m    =  4;
  const int   n    = 20;
  const float beta =  1.0f;
#else
  const int   m    =  2;
  const int   n    = 38;
  const float beta =  1.0f;
#endif
  filter::fir::raised_cosine<float> f(n);
  f.design(m, beta);
  for (auto x : f.coeff())
    std::cout << x << std::endl;

  std::cout << "--------------------\n";

  aligned_vector<float> b(n+m, 0.0f);
  std::copy     (f.coeff().begin(), f.coeff().end(), b.begin());
  std::transform(f.coeff().begin(), f.coeff().end(), b.begin()+4, b.begin()+4,
                 [](float x, float y) -> float { return -x+y; });
  for (auto x : b)
    std::cout << x << std::endl;

#else
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
#endif
  return EXIT_SUCCESS;
}
