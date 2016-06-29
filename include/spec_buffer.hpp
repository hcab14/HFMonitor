// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2016 Christoph Mayer
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
#ifndef _SPECTRUM_BUFFER_HPP_cm160627_
#define _SPECTRUM_BUFFER_HPP_cm160627_

#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include "Spectrum.hpp"

class spec_buffer : private boost::noncopyable {
public:
  typedef boost::shared_ptr<spec_buffer> sptr;
  typedef std::vector<float> vec_type;
  typedef vec_type::const_iterator const_iterator;

  static sptr make(int size_time,
		   int size_freq) {
    return sptr(new spec_buffer(size_time,
				size_freq));
  }

  int size_time()     const { return _size_time; }
  int current_index() const { return _current_index; }
  void increment_current_index() {
    _current_index += 1;
    _current_index %= size_time();
  }

  void insert(const frequency_vector<float>& fv) {
    for (int i=0, n=fv.size(); i<n; ++i) {
      _v[2*size_time()*i +   current_index()] = fv[i].second;
      _v[2*size_time()*i + 2*current_index()] = fv[i].second;
    }
    increment_current_index();
  }

  const_iterator get(int freq_index) const {
    return _v.begin() + 2*size_time()*freq_index + current_index();
  }

  virtual ~spec_buffer() {}

protected:
private:
  spec_buffer(int size_time,
	      int size_freq)
    : _v(2*size_time*size_freq)
    , _size_time(size_time)
    , _size_freq(size_freq)
    , _current_index(0) {}

  vec_type  _v;
  const int _size_time;
  const int _size_freq;
  int       _current_index;
} ;

#endif // _SPECTRUM_BUFFER_HPP_cm160627_

