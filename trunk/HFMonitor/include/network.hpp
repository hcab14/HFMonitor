// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _NETWORK_HPP_cm121227_
#define _NETWORK_HPP_cm121227_

#include <boost/asio/io_service.hpp>

namespace network {  
  // provides a (reference to a) single unique instance of boost::asio::io_service
  boost::asio::io_service& get_io_service() {
    static boost::asio::io_service service;
    return service;
  }

} // namespace network
#endif // _NETWORK_HPP_cm121227_
