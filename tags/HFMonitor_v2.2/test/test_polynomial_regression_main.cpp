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

#include "polynomial_regression.hpp"

int main()
{
  srand48(getpid());
  const size_t n(50);
  std::vector<double> y(n, 0);
  for (size_t i=0; i<n; ++i)
    y[i] = 0.5 + 2.*i + drand48();

  polynomial_regression pr(y, 2);
  std::cout << "x[0]= "   << pr.x()(0)   << std::endl;
  std::cout << "x[1]= "   << pr.x()(1)   << std::endl;
  std::cout << "q[0,0]= " << pr.q()(0,0) << std::endl;
  std::cout << "q[0,1]= " << pr.q()(0,1) << std::endl;
  std::cout << "q[1,0]= " << pr.q()(1,0) << std::endl;
  std::cout << "q[1,1]= " << pr.q()(1,1) << std::endl;
  std::cout << "chi2/dof= " << pr.chi2() << "/" << pr.dof() << std::endl;

  for (size_t i=0; i<n+50; i+=10) {
    const std::pair<double,double> yf(pr.eval(i));
    std::cout << "t= " << i << " " << yf.first <<  " " << yf.second << std::endl;
  }
}
