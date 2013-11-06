// -*- C++ -*-
// $Id$

#include <iostream>
#include "repack_buffer.hpp"

template<typename T>
struct dummy_processor {
  void process_samples(typename std::vector<T>::const_iterator beg,
		       typename std::vector<T>::const_iterator end,
		       boost::uint64_t counter) {
    std::cout << "counter= " << counter << " | ";
    std::copy(beg, end, typename std::ostream_iterator<T>(std::cout , " "));
    std::cout << std::endl;
  }
} ;

int main()
{
  const size_t N(6);
  typedef repack_buffer<int> buffer_type;
  buffer_type::vector_type v(N);
  buffer_type b(12, 0.5);
  dummy_processor<int> p;

  std::cout << "overlap = " << b.overlap() << std::endl;

  for (size_t i=0; i<500; ++i) {
    v[i%N] = i;
    if ((i%N) == (N-1)) {
      b.insert(&p, v.begin(), v.end());
    }
  }
}
