// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_PROXY_HPP_cm101026_
#define _FFT_PROXY_HPP_cm101026_

#include <iostream>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include "FFTResult.hpp"

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
  } ;
  
  typedef boost::shared_ptr<Base> Handle;
}

#endif // _FFT_PROXY_HPP_cm101026_
