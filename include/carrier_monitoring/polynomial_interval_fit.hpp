// -*- C++ -*-
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
#ifndef POLYNOMIAL_INTERVAL_FIT_HPP_cm131005_
#define POLYNOMIAL_INTERVAL_FIT_HPP_cm131005_

#include <algorithm>
#include <vector>

// #include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
// #include <boost/numeric/ublas/matrix_proxy.hpp>
// #include <boost/numeric/ublas/storage.hpp>
#include <boost/numeric/ublas/vector.hpp>
// #include <boost/numeric/ublas/vector_proxy.hpp>
// #include <boost/numeric/ublas/operation.hpp>

#include "InvertMatrix.hpp"

class polynomial_interval_fit {
public:
  typedef boost::numeric::ublas::slice slice;
  typedef boost::numeric::ublas::vector<double> vector_type;
  typedef boost::numeric::ublas::matrix<double> matrix_type;

  polynomial_interval_fit()
    : poly_degree_(1)
    , chi2_(0)
    , dof_(0) {}

  polynomial_interval_fit(size_t poly_degree,
			  const std::vector<double>& t_index_vector)
    : poly_degree_(poly_degree)
    , t_indices_(t_index_vector)
    , chi2_(0)
    , dof_(0) {
    assert(std::is_sorted(t_indices_));
    assert(t_indices_.size() >= 2);
    assert(t_indices_.size() >= 2);
  }

  size_t poly_degree() const { return poly_degree_; }

  template<typename T, typename U>
  bool fit(const std::vector<T>& t,
	   const std::vector<U>& y) {
    // by default use all measurements
    const std::vector<size_t> default_b(y.size(), 1);
    return fit(t, y, default_b);
  }
  template<typename T, typename U>
  bool fit(const std::vector<T>& t,
	   const std::vector<U>& y,
	   const std::vector<size_t>& b) {
    assert(t.size() == y.size());
    assert(t.size() == b.size());
    assert(std::is_sorted(t));
    if (b != b_) {
      typedef std::not_equal_to<size_t> neq_functor;
      const size_t length_y(std::count_if(b.begin(), b.end(),
					  std::binder1st<neq_functor>(neq_functor(), 0)));
#if 0
      std::cout << "TT " << length_y << std::endl;
#endif
      b_        = b;
      t_        = yf_  = y_ = vector_type(length_y, 0);
      make_slices_y(t, y, b);
      slices_x_ = make_slices_x(poly_degree_, slices_y_.size());
      x_        = vector_type((poly_degree_+1)*slices_y_.size(), 0);
      q_        = matrix_type(x_.size(), x_.size(), 0);
    }

    using namespace boost::numeric::ublas;
    // compute ata and aty
    const size_t poly_degree_plus_1(poly_degree_+1);
    const size_t nx(x_.size());
    vector_type aty(nx, 0);
    matrix_type ata(nx, nx, 0);

    size_t slice_counter(0);
    for (auto const& sy : slices_y_) {
      matrix_type a(sy.size(), poly_degree_plus_1, 0);
      for (size_t j(0), n(sy.size()); j<n; ++j) {
	const double x(t_(sy(j)) - t_indices_[slice_counter]);
#if 0
	std::cout << str(boost::format("T %3d %3d %5.2f %5.2f %5.2f\n")
			 % j % sy(j) % t_(sy(j)) % y_(sy(j)) % x);
#endif
	for (size_t k(0); k<poly_degree_plus_1; ++k)
	  a(j,k) = (k==0) ? 1 : x*a(j,k-1);
      }
#if 0
      std::cout << str(boost::format("T_ %3d\n") % y_.size());
#endif
      const slice& sx(slices_x_[slice_counter++]);
      vector_type _v(sx.size(), 0);
      matrix_type _m(sx.size(), sx.size(), 0);
      noalias(project(ata, sx, sx)) = axpy_prod(trans(a), a, _m, true);
      noalias(project(aty, sx))     = axpy_prod(project(y_, sy), a, _v, true);
    }

    // constraints
    const size_t nc(slices_y_.size()-1);
    // if degree>1 there are constraints on the 1st derivative
    const size_t nc_total(poly_degree_>1 ? 2*nc : nc);
    matrix_type ac(nc_total, nx, 0);
    for (size_t i(0); i<nc; ++i) {
      const double dx(t_indices_[i+1] - t_indices_[i]);
      for (size_t j(0); j<poly_degree_plus_1; ++j)
	ac(i,i*poly_degree_plus_1+j) = (j==0) ? 1 : dx*ac(i,i*poly_degree_plus_1+j-1);
      ac(i,(i+1)*poly_degree_plus_1) = -1;
    }

    if (poly_degree_>1) {
      for (size_t i(0); i<nc; ++i) {
	const double dx(t_indices_[i+1] - t_indices_[i]);
     	for (size_t j(1); j<poly_degree_plus_1; ++j)
     	  ac(nc+i,i*poly_degree_plus_1+j) = (j==1) ? 1 : dx*j/(j-1)*ac(nc+i,i*poly_degree_plus_1+j-1);
     	ac(nc+i,(i+1)*poly_degree_plus_1+1) = -1;
      }
    }

    // fill atac=(ata + constrains)
    matrix_type atac(nx+nc_total, nx+nc_total, 0);
    const slice sx(0,  1, nx);
    const slice sc(nx, 1, nc_total);
    project(atac, sx, sx) = ata;
    project(atac, sx, sc) = trans(ac);
    project(atac, sc, sx) = ac;

    // solve and compute the parameters x
    matrix_type qc(atac);
    if (!ublas_util::InvertMatrix(atac, qc))
      return false;

    noalias(q_) = project(qc, sx, sx);
    noalias(x_) = prod(q_, aty);

    // compute the fitted values yf
    slice_counter= 0;
    for (auto const& sy : slices_y_) {
      matrix_type a(sy.size(), poly_degree_plus_1, 0);
      for (size_t j(0), n(sy.size()); j<n; ++j) {
	const double x(t_(sy(j)) - t_indices_[slice_counter]);
	for (size_t k(0); k<poly_degree_plus_1; ++k)
	  a(j,k) = (k==0) ? 1 : x*a(j,k-1);
      }
      vector_type _y(sy.size(), 0);
      noalias(project(yf_, sy)) = axpy_prod(a, project(x_, slices_x_[slice_counter++]), _y, true);
    }

    // compute chi2 and d.o.f.
    chi2_  = norm_2(yf_ - y_);
    chi2_ *= chi2_;
    dof_   = y_.size() - nx + nc_total;

    return true;
  }

  const std::vector<double>& t_indices() const { return t_indices_; }
  const vector_type& t()  const { return t_;  }
  const vector_type& y()  const { return y_;  }
  const vector_type& yf() const { return yf_; }
  const vector_type& x()  const { return x_;  }
  const matrix_type& q()  const { return q_;  }

  // returns fit (value, rms) for given t
  std::pair<double, double> eval(double t) const {
    using namespace boost::numeric::ublas;
    const size_t i(find_index(t));
    const vector_type a(make_a(t, i));
    const slice& sx(slices_x_[i]);
    return std::make_pair(inner_prod(a, project(x_, sx)),
			  inner_prod(a, prod(project(q_,sx,sx), a)));

  }

  const double chi2() const { return chi2_; }
  const size_t dof() const { return dof_; }

protected:
  vector_type make_a(double t, size_t i) const {
    const double dt(t-t_indices_[i]);
    const size_t n(slices_x_[i].size());
    vector_type a(n, 0);
    for (size_t j(0); j<n; ++j)
      a(j) = (j==0) ? 1 : dt*a(j-1);
    return a;
  }
  size_t find_index(double t) const {
    return ((t >= t_indices_.back())
	    ? t_indices_.size()-2
	    : ((t <= t_indices_.front())
	       ? 0
	       : std::distance
	       (t_indices_.begin(),
		std::lower_bound(t_indices_.begin(), t_indices_.end(), t))-1));
  }

  // slices for the measurements
  template<typename T, typename U>
  void make_slices_y(const std::vector<T>& t,
		     const std::vector<U>& y,
		     const std::vector<size_t>& b) {
    assert(iv.size()>0);
    assert(t.size() == y.size());
    assert(t.size() == b.size());

    size_t counter(0);  // counts measurements t,y,b

    // skip measurements before the first interval
    while (t[counter] < t_indices_[0])
      ++counter;

    slices_y_ = std::vector<slice>(t_indices_.size()-1);
    for (size_t i(1), ib(0), it(0), n(t_indices_.size()); i<n; ++i) {
      size_t nb(0);
      for (; t[counter] < t_indices_[i] && counter<y.size(); ++counter) {
	if (b[counter]) {
	  ++nb;
 	  t_[it]   = t[counter];
	  y_[it++] = y[counter];
	}
      }
      slices_y_[i-1] = slice(ib, 1, nb);
      ib += nb;
    }
  }
  // slices for the model parameters
  static std::vector<slice> make_slices_x(size_t poly_degree, size_t num_slices) {
    std::vector<slice> sv(num_slices);
    for (size_t i(0); i<num_slices; ++i)
      sv[i] = slice((poly_degree+1)*i, 1, poly_degree+1);
    return sv;
  }

private:
  size_t poly_degree_;            // degree of polynomials
  std::vector<double> t_indices_; // interval boundaries (times)
  vector_type t_;                 // time of used measurements
  vector_type y_;                 // used measurements
  vector_type yf_;                // fitted values of used measurements
  std::vector<size_t> b_;         // b!=0 -> measurements is used
  std::vector<slice> slices_y_;   // slices for each interval
  std::vector<slice> slices_x_;   // slices for each set of polynomial pararameters
  vector_type x_;                 // esimated polynomial parameters
  matrix_type q_;                 // esimated convariances of polynomial parameters
  double chi2_;                   // chi^2 of fit
  size_t dof_;                    // number of degrees of freedom
} ;

#endif //POLYNOMIAL_INTERVAL_FIT_HPP_cm131005_
