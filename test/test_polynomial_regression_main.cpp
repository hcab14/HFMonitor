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

#include <iostream>
#include <fstream>

#include "polynomial_regression.hpp"

int main()
{
  std::vector<std::pair<double, double> > xy;
  for (size_t i=0; i<50; ++i)
    xy.push_back(std::make_pair(1.*i, 0.5+2.*i+0.5-drand48()));

  polynomial_regression pr(2, xy);
  std::cout << "x[0]= "   << pr.x(0) << std::endl;
  std::cout << "x[1]= "   << pr.x(1) << std::endl;
  std::cout << "q[0,0]= " << pr.q(0,0) << std::endl;
  std::cout << "q[0,1]= " << pr.q(0,1) << std::endl;
  std::cout << "q[1,0]= " << pr.q(1,0) << std::endl;
  std::cout << "q[1,1]= " << pr.q(1,1) << std::endl;
  std::cout << "chi2/dof= " << pr.chi2() << "/" << pr.dof() << std::endl;

  for (size_t i=0; i<100; i+=10) {
    std::cout << "t= " << i << " " << pr.eval(i) <<  " " << pr.eval_err(i) << std::endl;
  }
}
