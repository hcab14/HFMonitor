// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <iterator>
#include <vector>
#include <complex>

#include "IIR.hpp"

int main()
{
  std::vector<double> a, b;
  a.push_back( 1.0);
  a.push_back(-0.9);
  b.push_back( 0.0);
  b.push_back( 0.1);

  filter::iir<double, std::complex<double>, 2> filt(a,b);

  std::vector<double> v;
  for (int i=0; i<10; ++i)
    v.push_back(double(i));

  std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, " ")); 
  std::cout << std::endl;

  for (int i=0; i<20; ++i) {
    filter::iter_move<double, 10-2>::exec(v.begin(), v.begin()+1);
    ++v.back();
    std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, " ")); 
    std::cout << std::endl;
  }

  typedef filter::integrator<double> intd;
  typedef filter::integrator_modulo<double> intmd;

  intmd int1(2*M_PI);
  intmd int2(2*M_PI);

  filter::filter_pair<intmd, intmd> ip(int1, int2);

  for (int i=0; i<100000; ++i) {
    double r=drand48()-0.5;
    std::cout << "A " << i << " " << r << " " << ip.process(r) << " " << int1.process(r) << std::endl;
  }

}
