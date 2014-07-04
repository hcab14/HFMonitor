// -*- C++ -*-
// $Id$

#include <bzlib.h>

#include <stdlib.h>
#include <iostream>

#include "vector_compressor.hpp"

int main()
{
  size_t n(10000);
  std::vector<char> v(n,0);
  for (size_t i(0); i<50; ++i) {
    // v[lrand48()%n] =10+(lrand48()%4);
    // v[550+1*i] =1+(lrand48()%128);
    // v[777+i] = 1+(lrand48()%128);
  }

  vector_compressor vec_compressor;
  for (size_t i=0; i<1000; ++i) {
    // std::cout << "v  : "; std::copy(v.begin(), v.end(), std::ostream_iterator<int>(std::cout, " ")); std::cout << std::endl;
    const vector_compressor::vector_type cv(vec_compressor.compress(v));
    // std::cout << "cv : "; std::copy(cv.begin(), cv.end(), std::ostream_iterator<int>(std::cout, " ")); std::cout << std::endl;
    const vector_compressor::vector_type dcv(vec_compressor.decompress(cv));
    // std::cout << "dcv: ";std::copy(dcv.begin(), dcv.end(), std::ostream_iterator<int>(std::cout, " ")); std::cout << std::endl;
    std::cout << "Check : " << int(dcv == v) << std::endl;
    for (size_t j(0); j<15; ++j)
      v[lrand48()%n] =10+(lrand48()%4);
  }
  // std::copy(cv.begin(), cv.end(), std::ostream_iterator<int>(std::cout, " "));
  // std::cout << std::endl;

  // unsigned int dest_len(2*n);
  // std::vector<char> vc(2*n,0);
  // const int result = BZ2_bzBuffToBuffCompress(&vc[0], &dest_len, &v[0], n, 9, 0, 0);
  // std::cout << "bzip2: " << result << " " << dest_len << std::endl;
  // std::cout << "BZ2 compression factor = " << double(dest_len)/v.size() << " " << double(cv.size())/n<< std::endl;

  // std::copy(dcv.begin(), dcv.end(), std::ostream_iterator<int>(std::cout, " "));
  // std::cout << std::endl;

  return 0;

#if 0
  const size_t n(1000);
  std::vector<char> s(n,0);
  std::vector<char> sc(10*n,0);
  s[2] = 1;
  s[3] = 2;
  s[4] = 2;
  s[5] = 2;
  s[6] = 2;
  s[200] = 2;
  s[654] = 4;
  for (size_t i(0); i<n; ++i)
    s[i] = (!(i%2))*(1+0*lrand48());

  unsigned int dest_len(2*n);
  int result = BZ2_bzBuffToBuffCompress(&sc[0], &dest_len, &s[0], n, 9, 0, 0);
  std::cout << "bzip2: " << result << " " << dest_len << std::endl;
  std::cout << "sizeof(s) = " << sizeof(s) << std::endl;
#endif
}
