// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
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
#ifndef _PROCESSOR_HPP_cm121221_
#define _PROCESSOR_HPP_cm121221_

#include <string>
#include <complex>
#include <vector>
#include <boost/array.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "processor/service.hpp"

namespace processor {
  class processor : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<processor> sptr;

    enum {
      max_buffer_size = 1024*1024 // 1MB
    } ;
    typedef boost::array<char, max_buffer_size> data_buffer_type;
    
    processor() {}
    virtual ~processor() {};

    virtual void process(data_buffer_type::const_iterator begin,
			 data_buffer_type::const_iterator end) = 0;

  } ;

  class processor_iq : public processor {
  public:
    typedef boost::shared_ptr<processor_iq> sptr;
    typedef std::vector<std::complex<double> > const_iterator;
    virtual ~processor_iq() {};
    virtual void process_iq(const_iterator beg, const_iterator end) = 0;
  } ;

  class processor_txt : public processor {
  public:
    typedef boost::shared_ptr<processor_txt> sptr;
    virtual ~processor_txt() {};
  } ;

  class processor_png : public processor {
  public:
    typedef boost::shared_ptr<processor_png> sptr;
    virtual ~processor_png() {};
  } ;

}
#endif // _PROCESSOR_HPP_cm121221_
