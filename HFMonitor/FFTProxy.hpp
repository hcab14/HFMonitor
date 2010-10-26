// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_PROXY_HPP_cm101026_
#define _FFT_PROXY_HPP_cm101026_

#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include "FFTResult.hpp"

namespace Proxy {
  class Base : public boost::noncopyable {
  public:
    virtual ~Base() {}
    virtual void putResult(std::string resultKey, Result::Base::Handle result) {
      std::cout << "Proxy::Base::putResult [" << resultKey << "] " << result << std::endl;
    }
    virtual Result::Base::Handle getResult(std::string keyString) const = 0;
  } ;
  
  typedef boost::shared_ptr<Base> Handle;
}

#endif // _FFT_PROXY_HPP_cm101026_
