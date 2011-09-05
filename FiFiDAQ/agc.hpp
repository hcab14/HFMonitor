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
#ifndef _AGC_HPP_cm110809_
#define _AGC_HPP_cm110809_

#include <vector>
#include <complex>

namespace agc {

  // agc
  template <typename T>
  class agc {
  public:
    typedef T float_type;
    typedef typename std::complex<float_type> complex_type;
    typedef typename std::vector<complex_type> complex_vector_type;
    
    agc(double t_fall,
        double t_rise,
        double_t fs=1.)
      : t_fall_(t_fall)
      , t_rise_(t_rise)
      , fs_(fs)
      , alpha_(1./fs_*t_fall_)
      , beta_(1./fs_*t_rise_)
      , f_(float_type(1)) {}
    
    void reset() { f_=float_type(1); }

    double fs() const { return fs_; }
    double alpha() const { return alpha_; }
    double beta() const { return beta_; }

    void set_fs(double fs) { 
      fs_    = fs; 
      alpha_ = 1./fs_*t_fall_;
      beta_  = 1./fs_*t_rise_;      
    }

    complex_type process(complex_type s) {
      const float_type as(std::abs(s));
      f_ = (as < f_) 
        ? (1-alpha_)*f_ + alpha*as
        : (1-beta_ )*f_ + beta *as;
      return 0.5*s/f_;
    }    
  protected:
  private:
    double t_fall_;
    double t_rise_;
    double fs_;
    double alpha_;
    double beta_;
    float_type f_;
  } ;
} // namespace filter
#define _AGC_HPP_cm110809_
