// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <vector>
#include <stdlib.h>

#include "Filter.hpp"

int main()
{
  // make up time series
  using namespace boost::posix_time;
  using namespace boost::gregorian;
  std::vector<ptime> t;
  const double dt(0.01); // sec  
  const unsigned N(1*60/dt); // 1 minutes
  const double lambda(.1); // filter time constant / sec
  const double noise(dt*1000);
  for (unsigned u(0); u<N; ++u) {
    const double x(std::max(0., (1+u)*dt + 2*noise*(drand48()-1)));
    const ptime pt(date(2005,Jan,1), 
                   time_duration(23,59,30,x*time_duration::ticks_per_second()));
    t.push_back(pt);
  }

  Filter::PTimeLowPass f(dt, lambda);
  f.init(t[0]);

  // filter
  for (unsigned u(1); u<N; ++u) {
    f.update(t[u]);
    std::cout << "#T " << t[u] << " " << f.x() << " " 
              << t[u] - f.x() << " " << int(f.isInEquilibrium()) << std::endl;
  }

  // std::cout << microsec_clock::universal_time() << std::endl;
}
