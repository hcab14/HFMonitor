// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <fstream>

//#define USE_AXPY_PROD
#include "carrier_monitoring/polynomial_interval_fit.hpp"

int main()
{
  srand48(getpid());
  std::vector<double> v(100);
  for (size_t i=0; i<100; ++i) {
    const double x(0.01*(i-50.));
    v[i] = 1+0.3*drand48() - 2*x*x + 20*x*x*x;
  }

  std::vector<size_t> indices;
  indices.push_back( 0);
  indices.push_back( 0);
  indices.push_back( 0);
  indices.push_back( 0);
  indices.push_back(99);  

  const unsigned degree(4);

  polynomial_interval_fit p(v, indices, degree);

  for (size_t i=0; i<100; ++i) {
    std::cout << "E " << i << " " << v[i] << " " << p.eval(i) << " " << p.eval_error(i) << std::endl;
  }
  std::cout << "Chi2/dof = " << p.chi2() << " / " << p.dof() << std::endl;
}
