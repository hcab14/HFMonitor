// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
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
#ifndef _NETWORK_HPP_cm121227_
#define _NETWORK_HPP_cm121227_

#include <boost/asio/io_service.hpp>

namespace network {  
  //// provides a (reference to a) single unique instance of boost::asio::io_service
  boost::asio::io_service& get_io_service() {
    static boost::asio::io_service service;
    return service;
  }

} // namespace network
#endif // _NETWORK_HPP_cm121227_
