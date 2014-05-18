// -*- C++ -*-
// $Id$

#include <string>
#include <iostream>
#include <sstream>

#include "wave/writer.hpp"
#include "wave/reader.hpp"

void test_single(double s) {
  const size_t bits_per_sample(3*8);
  std::string data(1024, ' ');
  wave::detail::write_real_sample(data.begin(), bits_per_sample, s);
  const double sr = wave::detail::read_real_sample(data.begin(), bits_per_sample).second;
  std::cout << s << " " << sr << " " << sr/s-1 << std::endl;
}

int main()
{
  
  test_single(0.764342);
  test_single(-0.764342);
  test_single(0.0001);
  test_single(-.99999);

  return 1;
}
