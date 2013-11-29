// -*- C++ -*-
// $Id$

#include <math.h>

#include "demod/early_late_synch.hpp"

int main()
{
  const size_t N(1000);
  std::vector<int> bits(N);
  for (size_t i(0); i<N; ++i)
    bits[i] = (drand48() > 0.5);

  const size_t period(20);
  std::vector<int> signal(period*N);
  for (size_t i(0),j(0); i<N; ++i)
    for (size_t k(0); k<period; ++k,++j)
      signal[j] = 2*bits[i]-1;

  demod::early_late_synch els(period);

  for (size_t i=0; i<2; ++i)
    els.insert_signal(-1);
  for (size_t i(0),n(signal.size()),j(0); i<n; ++i) {
    els.insert_signal(signal[i]);
    if (els.bit_valid()) {
      std::cout << "bit: " << els.current_bit() << " " << bits[j++] << std::endl;
    }
  }
  
  return 1;
}
