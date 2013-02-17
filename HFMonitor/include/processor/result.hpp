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
#ifndef _RESULT_HPP_cm130217_
#define _RESULT_HPP_cm130217__

#include <iostream>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

namespace processor {

  // base class for time-stamped named results
  class result_base : private boost::noncopyable {
  public:
    typedef boost::posix_time::ptime ptime;

    typedef boost::shared_ptr<result_base> sptr;
    virtual ~result_base() {}

    virtual std::string name() const { return name_; }
    virtual ptime approx_ptime() const { return t_; }

    virtual std::string to_string() const { return name(); }    

    virtual std::ostream& dump_header(std::ostream& os) const { return os;  }
    virtual std::ostream& dump_data(std::ostream& os)   const { return os; }

    friend std::ostream& operator<<(std::ostream& os, const result_base& r) {
      return os << r.to_string();
    }
  protected:
    result_base(std::string name,  ptime t)
      : name_(name)
      , t_(t) {}

  private:    
    const std::string name_;
    const ptime       t_;
  } ;

} // namespace processor
#endif // _RESULT_HPP_cm130217_
