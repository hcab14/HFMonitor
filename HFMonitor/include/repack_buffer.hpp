// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id: IQBuffer.hpp 282 2013-07-07 20:01:57Z cmayer $
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
#ifndef _REPACK_BUFFER_HPP_cm130710_
#define _REPACK_BUFFER_HPP_cm130710_

#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "boost/cstdint.hpp"

template<typename T>
class repack_buffer {
public:
  typedef typename std::vector<T> vector_type;
  typedef typename vector_type::iterator iterator;
  typedef typename vector_type::const_iterator const_iterator;

  repack_buffer(size_t n,
                double overlap)
    : n_(n)
    , overlap_(overlap)
    , delta_n_(0)
    , i_(0) {
    resize();
  }

  double overlap() const { return overlap_; }
  int    delta_n() const { return delta_n_; }

  void resize(size_t n) {
    if (n_ != n) {
      n_ = n;
      resize();
    }
  }

  // calls PROCESSOR::process_samples(i1, i2);
  template<class PROCESSOR>
  void insert(PROCESSOR* p, const_iterator beg, const_iterator end) {
    // update parameters, if necessary
    resize();

    boost::uint64_t counter(0); // distance (in #samples) from beg

    // -1.0 < overlap  <= 0.0
    if (-1.0 < overlap_ && overlap_ <= 0.0) {
      int dist(0);
      while ((dist=std::distance(beg, end)) > 0) {
	const size_t n_samples(std::min(int(v_.size()-i_), dist));
	std::copy(beg, beg+n_samples, v_.begin()+i_);

	i_  += n_samples;
	beg += n_samples;
        counter += n_samples;
	if (i_ == int(v_.size())) {
	  // eject
	  if (NULL != p) {
	    p->process_samples(v_.begin(),
			       v_.begin() + n_,
                               counter);
	  }
	}
	i_ %= v_.size();
      }
    } else { // overlap \in [0.5 , 1.0)
      int dist(0);
      while ((dist=std::distance(beg, end)) > 0) {
	int n_samples(std::min(int(n_-i_), dist));
	if (n_samples > delta_n_)
	  n_samples = delta_n_;
	// nearest index for inserting data
	const int decision(delta_n_*((i_+n_samples)/delta_n_) - i_);	
	if (decision > 0) // inside the current samples
	  n_samples = std::min(n_samples,  decision);

	std::copy(beg, beg+n_samples, v_.begin()+i_);
	std::copy(beg, beg+n_samples, v_.begin()+i_+n_);
	
	i_  += n_samples;
	beg += n_samples;
        counter += n_samples;
	if ((i_ % delta_n_) == 0) {
	  // eject
	  if (NULL != p) {
	    p->process_samples(v_.begin()+i_,
			       v_.begin()+i_ + n_,
                               counter);
	  }
	}
	i_ %= n_;
      }
    }
  }

protected:
  // -1.0 < overlap <= 0.0 or 0.5 <= overlap < 1.0
  void resize() {
    if (overlap_ == 0.0) {
      if (v_.size() != n_) {
	v_.resize(n_);
	i_ = 0;
      }
    } else if (-1.0 < overlap_ && overlap_ < 0.0) {
      const size_t nv(size_t(n_/(-overlap_)));
      if (v_.size() != nv) {
	v_.resize(nv);
	i_ = 0;
      }
    } else if ( 0.5 <= overlap_ && overlap_ < 1.0) {
      if (v_.size() != 2*n_) {
	v_.resize(2*n_);      
	i_ = 0;
	delta_n_ = std::max(1, int((1-overlap_)*n_));
	// delta_n has to divide n_
	while (n_ % delta_n_) {
// 	  std::cout << "DD " << delta_n_ << " " << n_ << " " << (n_ % delta_n_) << std::endl;
	  --delta_n_;
	}
	overlap_ = 1-double(delta_n())/double(n_);
      }
    } else {
      throw std::runtime_error("repack_buffer::resize invalid overlap");
    }
  }
private:
  size_t n_;
  double overlap_;
  int    delta_n_;
  vector_type v_;
  int         i_; // current index
} ;

#endif // _REPACK_BUFFER_HPP_cm130710_
