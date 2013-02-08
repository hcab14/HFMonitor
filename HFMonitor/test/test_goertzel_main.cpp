// -*- c++ -*-

#include <iostream>
#include "filter/tracking_goertzel_filter.hpp"

typedef goertzel<std::complex<double> > goertzel_t;

int main()
{
  const double fs(1000);
  std::vector<std::complex<double> > v;
  const double f0(103.49608);
  tracking_goertzel_filter tf(fs, 100, 10, 0.1, 500);
  for (size_t i=0; i<fs*10000; ++i) {
    const double f(f0+0.1*i/fs/500);
    tf.update(exp(std::complex<double>(0, 2*M_PI*i/fs*f)) 
	      +0.01*std::complex<double>(drand48()-0.5, drand48()-0.5));
  }

  return 1;
}
