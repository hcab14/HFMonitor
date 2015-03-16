// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
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
#ifndef POLYNOMIAL_REGRESSION_HPP_cm130510_
#define POLYNOMIAL_REGRESSION_HPP_cm130510_


#include "carrier_monitoring/polynomial_interval_fit.hpp"

class polynomial_regression : public polynomial_interval_fit {
public:
  polynomial_regression(const std::vector<double>& t,
                        const std::vector<double>& v,
                        unsigned poly_degree)
    : polynomial_interval_fit(poly_degree, make_single_interval(v)) {
    ASSERT_THROW(t.size() == v.size());
    ASSERT_THROW(fit(t,v) == true);
  }
  
  virtual ~polynomial_regression() {}
  
private:
  static const std::vector<double>& make_single_interval(const std::vector<double>& v) {
    ASSERT_THROW(not v.empty());
    static std::vector<double> iv(2);
    iv[0] = 0;
    iv[1] = v.size()-1;
    return iv;
  }
} ;

#endif // POLYNOMIAL_REGRESSION_HPP_cm130510_
