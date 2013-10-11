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
    , slices_(make_slices(index_vector))
    , poly_degree_(poly_degree)
    , x_((poly_degree_+1)*slices_.size(), 0)
    , q_(x_.size(), x_.size(), 0) {
    using namespace boost::numeric::ublas;

    for (size_t i(0); i<y.size(); ++i)
      y_(i) = y[i];

    const size_t poly_degree_plus_1(poly_degree_+1);
    const size_t nx(x_.size());

    vector_type aty(nx);
    matrix_type ata(nx, nx);

    size_t ix(0);
    vector_type _v(poly_degree_plus_1, 0);
    matrix_type _m(poly_degree_plus_1, poly_degree_plus_1, 0);
    BOOST_FOREACH(const slice& sy, slices_) {
      matrix_type a(sy.size(), poly_degree_plus_1, 0);
      for (size_t j(0); j<sy.size(); ++j) {
	const double x(j);
	for (size_t k(0); k<poly_degree_plus_1; ++k)
	  a(j,k) = (k==0) ? 1 : x*a(j,k-1);
      }
      const slice sx(ix, 1, poly_degree_plus_1);
      ix += poly_degree_plus_1;

      project(ata, sx, sx) = axpy_prod(trans(a), a, _m, true);
      project(aty, sx)     = axpy_prod(project(y_, sy), a, _v, true);
    }

    // constraints
    const size_t nc(slices_.size()-1);
    matrix_type ac(nc, nx, 0);
    for (size_t i(0); i<nc; ++i) {
      const double x(slices_[i].size());
      for (size_t j(0); j<poly_degree_plus_1; ++j) {
	ac(i,i*poly_degree_plus_1+j) = (j==0) ? 1 : x*ac(i,i*poly_degree_plus_1+j-1);
      }
      ac(i,(i+1)*poly_degree_plus_1) = -1;
    }

    matrix_type atac(nx+nc, nx+nc, 0 );

    const slice sx(0,  1, nx);
    const slice sc(nx, 1, nc);
    project(atac, sx, sx) = ata;
    project(atac, sx, sc) = trans(ac);
    project(atac, sc, sx) = ac;

    std::cout << "ata= " << atac << std::endl;
    std::cout << "aty= " << aty << std::endl;
    matrix_type qc(atac);
    ublas_util::InvertMatrix(atac, qc);
    q_ = project(qc, sx, sx);

    x_ =  prod(q_, aty);
    // yf
    ix=0;
    BOOST_FOREACH(const slice& sy, slices_) {
      matrix_type a(sy.size(), poly_degree_plus_1, 0);
      for (size_t j(0); j<sy.size(); ++j) {
	const double x(j);
	for (size_t k(0); k<poly_degree_plus_1; ++k)
	  a(j,k) = (k==0) ? 1 : x*a(j,k-1);
      }
      const slice sx(ix, 1, poly_degree_plus_1);
      ix += poly_degree_plus_1;

      vector_type _y(sy.size(), 0);
      project(yf_, sy) = axpy_prod(a, project(x_, sx), _y, true);
    }    
  }

  ~polynomial_interval_fit() {}

  double eval(double t) const {
    using namespace boost::numeric::ublas;
    const size_t i(find_index(t));
    const double x(t-indices_[i]);

    const slice sx(i*poly_degree_+1, 1, poly_degree_+1);
    vector_type a(sx.size(), 0);
    for (size_t j(0); j<poly_degree_+1; ++j)
      a(j) = (j==0) ? 1 : x*a(j-1);

    std::cout << "eval: t,i,x,a= " 
	      << t << " "
	      << i << " "
	      << x << " "
	      << a
	      << std::endl;
    return inner_prod(a, project(x_, sx));
  }    
  double eval_error(size_t i) const {    

    return 0;
  }

  const double chi2() const { return 0; }
  const size_t dof() const { return 0; }

protected:
  size_t find_index(double t) const {
    return ((t >= indices_.back())
	    ? indices_.size()-2
	    : ((t <= indices_.front())
	       ? 0
	       : std::distance(indices_.begin(), std::lower_bound(indices_.begin(), indices_.end(), t))-1));
  }
  static std::vector<slice> make_slices(const index_vector_type& iv) {
    assert(iv.size()>0);
    std::vector<slice> sv(iv.size()-1);
    for (size_t i(1); i<iv.size(); ++i)
      sv[i-1] = slice(iv[i-1], 1, iv[i]-iv[i-1]);
    return sv;
  }

private:
  vector_type y_;
  vector_type yf_;
  index_vector_type indices_;
  std::vector<slice> slices_;
  size_t poly_degree_;
  vector_type x_;
  matrix_type q_;
} ;

#endif //POLYNOMIAL_INTERVAL_FIT_HPP_cm131005_

