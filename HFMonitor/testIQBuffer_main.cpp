// -*- C++ -*-
// $Id$
#include "IQBuffer.hpp"

int main() {
  IQBuffer b(4, 0.75);
  for (size_t u(0); u<100; ++u) 
    b.insert(std::complex<double>(u,0));
}
