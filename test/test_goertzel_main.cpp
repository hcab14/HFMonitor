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
#include <iostream>
#include "run.hpp"
#include "filter/tracking_goertzel_filter.hpp"

int main() {
  LOGGER_INIT("./Log", "test_goertzel");
  const double fs(1000);
  std::vector<std::complex<double> > v;
  const double f0(103);
  tracking_goertzel_filter::sptr tf(tracking_goertzel_filter::make(fs, 100, 10, 0.1, 500, 5));
  for (size_t i=0; i<fs*10000; ++i) {
    const double f(f0-0.01*i/fs/500);
    tf->update(exp(std::complex<double>(0, 2*M_PI*i/fs*f)) 
	       +0.1*std::complex<double>(drand48()-0.5, drand48()-0.5));
  }
  return 0;
}
