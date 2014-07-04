// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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

#include "logging.hpp"
#include "carrier_monitoring/polynomial_interval_fit.hpp"

void test1() {
  srand48(getpid());
  const size_t n(100);
  std::vector<double> v(n);
  std::vector<size_t> b(n, 1);
  for (size_t i(0); i<n; ++i) {
    v[i] = 1+0.99*drand48() + cos(2*0.01*i*2*M_PI);
    b[i] = (i%4 == 0);
    if ((i%4) ==0)
      v[i] += 5;
  }

  std::vector<size_t> indices;
  indices.push_back( 0);
  indices.push_back(25);
  indices.push_back(50);
  indices.push_back(75);
  indices.push_back(99);  

  const unsigned poly_degree(3);

  polynomial_interval_fit p(poly_degree, indices);
  
  if (!p.fit(v, b)) {
    std::cerr << "fit failed" << std::endl;
    return;
  }
  
  for (int i=0; i<100; ++i) {
     const std::pair<double,double> vf(p.eval(i));
     std::cout << "E " << i << " " << v[i] << " " << vf.first << " " << vf.second << std::endl;
  }

  for (size_t i=0; i<p.y().size(); ++i) {
    const std::pair<double,double> vf(p.eval(p.t()(i)));
    std::cout << "X " << p.t()(i) << " " << p.y()(i) << " " << p.yf()(i) << " " << vf.first << " " << vf.second << std::endl;
  }
  std::cout << "Chi2/dof = " << p.chi2() << " / " << p.dof() << std::endl;
}

void test2() {
  std::ifstream ifs("test/spec.dat");

  const size_t n(1000);
  std::vector<double> v(n);
  std::vector<size_t> b(n, 1);
  double f(0), dummy(0);
  for (size_t i(0); i<n; ++i) {
    ifs >> f >> dummy >> v[i] >> dummy;
    std::cout << "S " << i << " " << f <<  " " << v[i] << " "<< b[i] << std::endl;
  }

  std::vector<size_t> indices;
  for (size_t i=0; i<10; ++i)
    indices.push_back((i*n)/10);
  indices.push_back(n-1);

  for (size_t i=0; i<indices.size(); ++i)
    std::cout << "I " << indices[i] << std::endl;

  const unsigned poly_degree(2);

  polynomial_interval_fit p(poly_degree, indices);

  for (size_t l(0); l<2000; ++l) {
    if (!p.fit(v, b)) {
      std::cerr << "fit failed" << std::endl;
      return;
    }
    size_t nchanged(0);
    for (size_t i(0); i<n; ++i) {
      const std::pair<double,double> vf(p.eval(i));
      const bool c(v[i]-vf.first > 10.);
      if (c==b[i]) {
        std::cout << "CC: " << i << " " << v[i] - vf.first << std::endl;
      }
      nchanged += (c==b[i]);
      b[i] = !c;
    }
    std::cout << "nchanged,Chi2/dof = "<< nchanged << " "  << p.chi2() << " / " << p.dof() << std::endl;
    if (0 == nchanged)
      break;
  }

  for (size_t i(0); i<n; ++i) {
    const std::pair<double,double> vf(p.eval(i));
    std::cout << "X " << i << " " << v[i] << " " << vf.first << " " << vf.second << std::endl;
  }
}

int main()
{
   LOGGER_INIT("./Log", "test_pif");
//    test1();
   test2();
}
