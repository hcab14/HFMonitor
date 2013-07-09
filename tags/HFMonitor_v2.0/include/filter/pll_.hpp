// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
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
#ifndef _PLL_HPP_cm130122_
#define _PLL_HPP_cm130122_

#include <vector>
#include <complex>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

namespace filter {
  class pll : private boost::noncopyable {
  public:
    typedef boost::shared_ptr<pll> sptr;

    virtual ~pll() {}

    static sptr make() {
    }

    void update(std::complex<double> z) {
      // phase update
      theta_ += f1_*ts_;
      // phase detector
      const double ud_old(ud_);
      ud_ = std::arg(z*std::exp(std::complex<double>(0, -theta_)));
      // loop filter (1 + s*tau1) / (s*tau2)
      uf = 
    }

  protected:
  private:
    pll(double xi, double dwl, double fc, double fs)
      : ud_(0)
      , uf_(0)
      , f1_(2*M_PI*fc)
      , theta_(-f1_/fs) {
      const double wl(dwl*2*M_PI);
      const double wn(wl/2/xi);
      const double k0(1);
      const double kd(1);
      const double tau1(k0*kd/(wn*wn));
      const double tau2(xi*2/wn);      
      a_[0] =  1;
      a_[1] = -1;
      b_[0] = .5/fs/tau1*(1+tan(.5/fs/tau2));
      b_[1] = .5/fs/tau1*(1-tan(.5/fs/tau2));
    }

    double a_[2];
    double b_[2];
    double ud_;
    double uf_;
    double f1_;
    double theta_;
  } ;

} // namespace filter

#endif // _FIR_HPP_cm110527_
