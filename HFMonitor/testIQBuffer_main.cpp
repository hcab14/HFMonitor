// -*- C++ -*-
// $Id$
#include "IQBuffer.hpp"

int main() {
  Internal::ModuloCounter<size_t> m(4);

  for (size_t u(0); u<10; ++u) {
    ++m;
    std::cout << u << " " << m << std::endl;
  }
  for (size_t u(0); u<10; ++u) {
    --m;
    std::cout << u << " " << m << std::endl;
  }

  IQBuffer b(7, 0.5);
  for (size_t u(0); u<100; ++u) {
    std::cout << "------------------------------\n";
    b.insert(&b, std::complex<double>(u,0));
    const IQBuffer::Samples s(b.samples());
    std::cout << "--- lastIQ ";
    std::copy(s.begin(),s.end(), std::ostream_iterator<std::complex<double> >(std::cout, " "));
    std::cout << std::endl;
  }
}
