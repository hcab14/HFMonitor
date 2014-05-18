// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2014 Christoph Mayer
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
#ifndef _VECTOR_COMPRESSOR_cm140413_
#define _VECTOR_COMPRESSOR_cm140413_

#include <vector>

class vector_compressor {
public:
  typedef std::vector<char> vector_type;
  typedef vector_type::iterator iterator;
  typedef vector_type::const_iterator const_iterator;

  vector_compressor() {}
  ~vector_compressor() {}

  vector_type compress(const vector_type& v) const {
    // compressed vector
    vector_type cv(v.size()/10, 0);
    // i ... current entry of v
    // j ... entry in cv indicating the length of the following segment
    // k ... current entry in cv where the positive entires are stored
    iterator j(cv.begin()), k(j);
    *j = 0;
    for (const_iterator i(v.begin()), end(v.end()); i!=end; ++i) {
      // resize cv if needed, keeping the iterators j,k valid
      const size_t dj(std::distance(cv.begin(), j));
      const size_t dk(std::distance(cv.begin(), k));
      if (dj + 3 > cv.size() || dk + 3 > cv.size()) {
	cv.resize(1 + 12*cv.size()/10, 0);
	j = cv.begin() + dj;
	k = cv.begin() + dk;
      }
      //          (a) *i==0
      if (*i == 0) {
	if (*j == -127) *(k=++j)   = 0;
	if (*j > 0)     *(j=++k) = 0;
	--(*j);
      } else { // (b) *i!=0	
	if (*j < 0 || *j == 127) // start a new block of data
	  *(j=++k) = 0;
	*++k = *i;
	++(*j);
      }
    }
    cv.resize(std::distance(cv.begin(), k+1));
    return cv;
  }

  vector_type decompress(const vector_type& cv) const {
    // decompressed vector
    vector_type v;
    for (const_iterator i(cv.begin()), end(cv.end()); i!=end;) {
      if (*i < 0) { // (a) (-*i) times 0
	const size_t m(-*i++);
	if (m > 32) { // optimization to void calling push_back
	  v.resize(v.size() + m, 0);
	} else {
	  for (size_t j(0); j<m; ++j)
	    v.push_back(0);
	}
      } else {      // (b) (*i) next values
	const size_t m(*i++);
	if (m > 32) { // optimization to void calling push_back
	  v.resize(v.size() + m, 0);
	  std::copy(i, i+m, v.end()-m);
	  i += m;
	} else {
	  for (size_t j(0); j<m && i!=end; ++j) 
	    v.push_back(*i++);
	}
      }
    }
    return v;
  }

protected:
private:
} ;

#endif // _VECTOR_COMPRESSOR_cm140413_
