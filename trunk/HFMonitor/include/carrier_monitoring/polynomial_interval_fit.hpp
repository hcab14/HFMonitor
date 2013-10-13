// -*- C++ -*-
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
#ifndef POLYNOMIAL_INTERVAL_FIT_HPP_cm131005_
#define POLYNOMIAL_INTERVAL_FIT_HPP_cm131005_

#include <algorithm> 
#include <vector>

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/operation.hpp>

#include <boost/foreach.hpp>

#include "InvertMatrix.hpp"

class polynomial_interval_fit {
public:
  typedef boost::numeric::ublas::slice slice;
  typedef boost::numeric::ublas::vector<double> vector_type;
  typedef boost::numeric::ublas::matrix<double> matrix_type;
  typedef std::vector<size_t> index_vector_type;

  polynomial_interval_fit(const std::vector<double>& y,
			  const index_vector_type& index_vector,
			  size_t poly_degree)
    : y_(y.size(), 0)
    , yf_(y.size(), 0)
    , indices_(index_vector)
    , slices_y_(make_slices_y(index_vector))
    , slices_x_(make_slices_x(poly_degree, slices_y_.size()))
    , poly_degree_(poly_degree)
    , x_((poly_degree_+1)*slices_y_.size(), 0)
    , q_(x_.size(), x_.size(), 0)
    , chi2_(0)
    , dof_(0) {
    using namespace boost::numeric::ublas;

    std::copy(y.begin(), y.end(), y_.begin());

    // compute ata and aty
    const size_t poly_degree_plus_1(poly_degree_+1);
    const size_t nx(x_.size());
    vector_type aty(nx);
    matrix_type ata(nx, nx);

    size_t slice_counter(0);
    BOOST_FOREACH(const slice& sy, slices_y_) {
      matrix_type a(sy.size(), poly_degree_plus_1, 0);
      for (size_t j(0); j<sy.size(); ++j) {
	const double x(j);
	for (size_t k(0); k<poly_degree_plus_1; ++k)
	  a(j,k) = (k==0) ? 1 : x*a(j,k-1);
      }
      const slice& sx(slices_x_[slice_counter++]);
      vector_type _v(sx.size(), 0);
      matrix_type _m(sx.size(), sx.size(), 0);
      project(ata, sx, sx) = axpy_prod(trans(a), a, _m, true);
      project(aty, sx)     = axpy_prod(project(y_, sy), a, _v, true);
    }

    // constraints
    const size_t nc(slices_y_.size()-1);
    const size_t nc_total(poly_degree_>1 ? 2*nc : nc);
    matrix_type ac(nc_total, nx, 0);
    for (size_t i(0); i<nc; ++i) {
      const double dx(slices_y_[i].size());
      for (size_t j(0); j<poly_degree_plus_1; ++j)
	ac(i,i*poly_degree_plus_1+j) = (j==0) ? 1 : dx*ac(i,i*poly_degree_plus_1+j-1);
      ac(i,(i+1)*poly_degree_plus_1) = -1;
    }

    if (poly_degree>1) {
      for (size_t i(0); i<nc; ++i) {
     	const double dx(slices_y_[i].size());
     	for (size_t j(1); j<poly_degree_plus_1; ++j)
     	  ac(nc+i,i*poly_degree_plus_1+j) = (j==1) ? 1 : dx*j/(j-1)*ac(nc+i,i*poly_degree_plus_1+j-1);
     	ac(nc+i,(i+1)*poly_degree_plus_1+1) = -1;
      }      
    }

    // fill atac=(ata + constrains)
    matrix_type atac(nx+nc_total, nx+nc_total, 0 );
    const slice sx(0,  1, nx);
    const slice sc(nx, 1, nc_total);
    project(atac, sx, sx) = ata;
    project(atac, sx, sc) = trans(ac);
    project(atac, sc, sx) = ac;

    // solve and compute the parameters x
    matrix_type qc(atac);
    ublas_util::InvertMatrix(atac, qc);
    q_ = project(qc, sx, sx);
    x_ =  prod(q_, aty);

    // computer the fitted values yf
    slice_counter= 0;
    BOOST_FOREACH(const slice& sy, slices_y_) {
      matrix_type a(sy.size(), poly_degree_plus_1, 0);
      for (size_t j(0); j<sy.size(); ++j) {
	const double x(j);
	for (size_t k(0); k<poly_degree_plus_1; ++k)
	  a(j,k) = (k==0) ? 1 : x*a(j,k-1);
      }
      vector_type _y(sy.size(), 0);
      project(yf_, sy) = axpy_prod(a, project(x_, slices_x_[slice_counter++]), _y, true);
    }

    // compute chi2 and d.o.f.
    const vector_type dy(yf_ - y_);
    chi2_ = inner_prod(dy, dy);
    dof_  = dy.size() - nx + nc_total;
  }

  ~polynomial_interval_fit() {}

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
    const double dt(t-indices_[i]);
    const size_t n(slices_x_[i].size());
    vector_type a(n, 0);
    for (size_t j(0); j<n; ++j)
      a(j) = (j==0) ? 1 : dt*a(j-1);
    return a;
  }
  size_t find_index(double t) const {
    return ((t >= indices_.back())
	    ? indices_.size()-2
	    : ((t <= indices_.front())
	       ? 0
	       : std::distance(indices_.begin(), std::lower_bound(indices_.begin(), indices_.end(), t))-1));
  }
  static std::vector<slice> make_slices_y(const index_vector_type& iv) {
    assert(iv.size()>0);
    std::vector<slice> sv(iv.size()-1);
    for (size_t i(1); i<iv.size(); ++i)
      sv[i-1] = slice(iv[i-1], 1, iv[i]-iv[i-1]);
    return sv;
  }
  static std::vector<slice> make_slices_x(size_t poly_degree, size_t num_slices) {
    std::vector<slice> sv(num_slices);
    for (size_t i(0); i<num_slices; ++i)
      sv[i] = slice((poly_degree+1)*i, 1, poly_degree+1);
    return sv;
  }

private:
  vector_type y_;
  vector_type yf_;
  const index_vector_type indices_;
  const std::vector<slice> slices_y_;
  const std::vector<slice> slices_x_;
  size_t poly_degree_;
  vector_type x_;
  matrix_type q_;
  double chi2_;
  size_t dof_;
} ;

#endif //POLYNOMIAL_INTERVAL_FIT_HPP_cm131005_

