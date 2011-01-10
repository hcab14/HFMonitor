// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <vector>
#include <stdlib.h>

#include "Filter.hpp"

void test1() {
  // make up time series
  using namespace boost::posix_time;
  using namespace boost::gregorian;
  std::vector<ptime> t;
  const double dt(2720./500e3); // sec  
  const unsigned N(24*3600/dt); // 24 h
  const double lambda(60.); // filter time constant / sec
  const double noise(dt*.2);
  const double offset_ppm(4);
  for (unsigned u(0); u<N; ++u) {
    const double x(std::max(0., (1+u)*dt*(1-1e-6*offset_ppm) + 2*noise*(drand48()-1)));
    const ptime pt(date(2005,Jan,1), 
                   time_duration(0,0,0,x*time_duration::ticks_per_second()));
    t.push_back(pt);
  }

  Filter::PTimeLowPass f(dt, lambda);
  f.init(t[0], t[0]);

  Filter::Cascaded<ptime> ff;
  ff.add(Filter::PTimeLowPass::make(dt, 600.));
  ff.add(Filter::PTimeLowPass::make(dt, 60.));
  ff.init(t[0], t[0]);

  // filter
  for (unsigned u(1); u<N; ++u) {
    f.update(t[u]);
    if ( (u % 661765) != 1)
      ff.update(t[u],t[u]);
    if ( (u % 368) == 1)
      std::cout << "#T " << t[u] << " " << f.x() << " " 
                << t[u] - f.x() << " " << int(f.isInEquilibrium()) << " "
                << ff.x()<< std::endl;
  }

  // std::cout << microsec_clock::universal_time() << std::endl;
}

int main()
{
  // make up time series
  using namespace boost::posix_time;
  using namespace boost::gregorian;
  
  ptime t,t0;
  std::cin >> t0;
  Filter::Cascaded<ptime> ff;
  const double dt(0.00544);
  time_duration dttd(0,0,0, dt * time_duration::ticks_per_second());
  ff.add(Filter::PTimeLowPass::make(dt, 1.0));
  ff.add(Filter::PTimeLowPass::make(dt, 1.0));
  ff.init(t0,t0);
  
  while (std::cin >> t) {
    bool interp(false);
    if (std::abs(double((t-t0).ticks())/double(time_duration::ticks_per_second())-dt) > 0.02*dt) {
      ff.update(ff.x() + dttd,
                ff.x() + dttd);
      interp= true;
    } else {
      ff.update(t,t);
    }
    if (!interp && std::abs((t - ff.x()).ticks()) > 0.02*dttd.ticks())
        ff.init(t,t);
    std::cout << "#T " << t << " " << ff.x() << " " 
              << t - ff.x() << " " << int(ff.isInEquilibrium()) 
              << std::endl;
    t0= t;
  }
}
