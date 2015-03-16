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
  const double fs(1000); // 1000 Hz

  const double f0(1);
  tracking_goertzel_filter::sptr tf(tracking_goertzel_filter::make(fs, 1, 4, 0.125, 40, 2));
  for (size_t i=0; i<fs*10000; ++i) {
    const double t(i/fs);

    // saw tooth 
//     const double f(f0 + std::abs(std::fmod(df*t, 0.02)-0.01));
    const double f(f0 + 0.001*std::cos(t/10000.));

    tf->update( exp(std::complex<double>(0, 2*M_PI*t*f))
	       +0.9*std::complex<double>(drand48()-0.5, drand48()-0.5));
    if (tf->state_updated()) {
      const detail::value_and_error& f_est =  tf->estimated_f_Hz();
      std::cout << "F " << std::scientific
                << t << " " << f << " "
                << t << " " << tf->delta_time_sec() << " " << f_est.value() << " " << f_est.rms_value()
                << std::endl;
    }
  }
  return 0;
}
