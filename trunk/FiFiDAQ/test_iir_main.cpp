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

  iir<double, std::complex<double>, 2> filt(a,b);

  std::vector<double> v;
  for (int i=0; i<10; ++i)
    v.push_back(double(i));

  std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, " ")); 
  std::cout << std::endl;

  for (int i=0; i<20; ++i) {
    iter_move<double, 10-2>::exec(v.begin(), v.begin()+1);
    ++v.back();
    std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, " ")); 
    std::cout << std::endl;
  }

}
