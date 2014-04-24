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
#ifndef OUTLIER_DETECTOR_HPP_cm130522_
#define OUTLIER_DETECTOR_HPP_cm130522_

#include <vector>

#include "polynomial_regression.hpp"

class outlier_detector {
public:
  typedef boost::numeric::ublas::matrix<double> matrix_type;
  typedef boost::numeric::ublas::vector<double> vector_type;
  
  outlier_detector(size_t n, // max. degree of polynomial
                   const std::vector<std::pair<double, double> >& xy) // (x,y) pair
    : n_(n)
    , x_(n)
    , q_(n,n)
    , chi2_(0)
    , dof_(0) {
  }

  double eval(double t) const {
    const vector_type a(make_a(t));
    return inner_prod(a,x_);    
  }
  double eval_err(double t) const {
    const vector_type a(make_a(t));
    return inner_prod(a, vector_type(prod(q_,a)));    
  }

  size_t n() const { return n_; }
  double x(size_t index) const { return x_(index); }
  double q(size_t i, size_t j) const { return q_(i,j); }
  double chi2() const { return chi2_; }
  size_t dof() const { return dof_; }

protected:
  vector_type make_a(double t) const {
    vector_type a(n());
    double t_acc(1);
    for (size_t i(0); i<n(); ++i) {
      a(i) = t_acc;
      t_acc *= t;
    }
    return a;
  }

private:
  size_t      n_;  // order of polynomial
  vector_type x_;  // parameters to be estimated
  matrix_type q_;  // variance-covariance matrix for x
  double      chi2_;
  size_t      dof_;
} ;

#endif // OUTLIER_DETECTOR_HPP_cm130522_
