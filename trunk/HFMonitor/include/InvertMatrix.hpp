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
// cf. http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?LU_Matrix_Inversion
#ifndef INVERT_MATRIX_HPP
#define INVERT_MATRIX_HPP

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "logging.hpp"

/* Matrix inversion routine.
   Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
namespace ublas_util {  
  template<class T>
  bool InvertMatrix(const boost::numeric::ublas::matrix<T>& input, 
                    boost::numeric::ublas::matrix<T>& inverse) {
    using namespace boost::numeric::ublas;
    typedef permutation_matrix<std::size_t> pmatrix;
    try {
      // create a working copy of the input
      matrix<T> A(input);
      // create a permutation matrix for the LU-factorization
      pmatrix pm(A.size1());
      // perform LU-factorization
      if (0 != lu_factorize(A, pm))
        return false;
      // create identity matrix of "inverse"
      inverse.resize(A.size1(), A.size1());
      inverse.assign(identity_matrix<T>(A.size1()));
      // backsubstitute to get the inverse
      lu_substitute(A, pm, inverse);
    } catch (const std::exception& e) {
      LOG_ERROR(e.what());
      return false;
    } catch (...) {
      LOG_ERROR("InvertMatrix: unexpected exception raised");
      return false;
    }
    return true;
  }
}
#endif //INVERT_MATRIX_HPP
