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
#ifndef _FFT_PROXY_HPP_cm101026_
#define _FFT_PROXY_HPP_cm101026_

#include <iostream>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include "FFTProcessor/Result.hpp"

namespace Proxy {
  class Base : public boost::noncopyable {
  public:
    typedef boost::posix_time::ptime ptime;
    virtual ~Base() {}
    virtual void putResult(std::string resultKey, Result::Base::Handle result) {
      LOG_INFO(str(boost::format("Proxy::Base::putResult [%s] %d") % resultKey % result));
    }
    virtual Result::Base::Handle getResult(std::string keyString) const = 0;
    virtual ptime getApproxPTime() const = 0;
    virtual double volt2dbm(double volt) const = 0;
    virtual double rms_dbm() const = 0;
  } ;
  
  typedef boost::shared_ptr<Base> Handle;
}

#endif // _FFT_PROXY_HPP_cm101026_
