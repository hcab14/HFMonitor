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
#ifndef _DB_BASE_HPP_cm140413_
#define _DB_BASE_HPP_cm140413_

#include <iostream>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>

namespace db {
  // base class for data bases
  class base : private boost::noncopyable {
  public:
    typedef boost::shared_ptr<base> sptr;
    virtual ~base() {}
    virtual boost::int64_t last_insert_rowid() const = 0;
  protected:
  private:
  } ;

} // namespace db

#endif // _DB_BASE_HPP_cm140413_
