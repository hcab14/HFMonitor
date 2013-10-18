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

#include <vector>
#include <iostream>
#include <fstream>

//#define USE_AXPY_PROD
#include "carrier_monitoring/polynomial_interval_fit.hpp"

int main()
{
  srand48(getpid());
  const size_t n(100);
  std::vector<double> v(n);
  std::vector<size_t> b(n, 1);
  for (size_t i(0); i<n; ++i) {
    v[i] = 1+0.1*drand48() + cos(2*0.01*i*2*M_PI);
    b[i] = (i%4 == 0);
  }

  std::vector<size_t> indices;
  indices.push_back( 0);
  indices.push_back(25);
  indices.push_back(50);
  indices.push_back(75);
  indices.push_back(99);  

  const unsigned poly_degree(2);

  polynomial_interval_fit p(poly_degree, indices);
  
  if (!p.fit(v, b)) {
    std::cerr << "fit failed" << std::endl;
    return 0;
  }
  
  for (int i=0; i<100; ++i) {
     const std::pair<double,double> vf(p.eval(i));
     std::cout << "E " << i << " " << v[i] << " " << vf.first << " " << vf.second << std::endl;
  }

  for (int i=0; i<p.y().size(); ++i) {
    const std::pair<double,double> vf(p.eval(p.t()(i)));
    std::cout << "X " << p.t()(i) << " " << p.y()(i) << " " << p.yf()(i) << " " << vf.first << " " << vf.second << std::endl;
  }
  std::cout << "Chi2/dof = " << p.chi2() << " / " << p.dof() << std::endl;
}
