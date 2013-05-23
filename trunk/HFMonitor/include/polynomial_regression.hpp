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
#ifndef POLYNOMIAL_REGRESSION_HPP_cm130510_
#define POLYNOMIAL_REGRESSION_HPP_cm130510_

#include <vector>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "InvertMatrix.hpp"

class polynomial_regression {
public:
  typedef boost::numeric::ublas::matrix<double> matrix_type;
  typedef boost::numeric::ublas::vector<double> vector_type;
  
  polynomial_regression(size_t n, // max. degree of polynomial
                        const std::vector<std::pair<double, double> >& xy) // (x,y) pair
    : n_(n)
    , x_(n)
    , q_(n,n)
    , chi2_(0)
    , dof_(0) {
    using namespace boost::numeric::ublas;
    const size_t n_meas(xy.size());
    matrix_type a(n_meas, n);
    vector_type y(n_meas);
    for (size_t i(0); i<n_meas; ++i) {
      const double x(xy[i].first);
      y(i) = xy[i].second;
      double t_acc(1);
      for (size_t j(0); j<n_; ++j) {
        a(i,j) = t_acc;
        t_acc *= x;
      }
    }
    const matrix_type ata(prod(trans(a),a));
    ASSERT_THROW(ublas_util::InvertMatrix(ata, q_) == true);
    x_ = prod(q_, vector_type(prod(trans(a),y)));
    const vector_type dy(y - prod(a,x_));
    chi2_ = inner_prod(dy, dy);
    dof_ = xy.size() - n_;
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

#endif // POLYNOMIAL_REGRESSION_HPP_cm130510_
