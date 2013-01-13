// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _CLIENT_HPP_cm121230_
#define _CLIENT_HPP_cm121230_

#include <iostream>
#include <set>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include "logging.hpp"
#include "network/broadcaster/directory.hpp"
#include "network/client/client_base.hpp"
#include "network/client/service_net.hpp"
#include "network/protocol.hpp"
#include "processor.hpp"

// generic base class for connecting to a broadcaster
template<typename PROCESSOR>
class client : public client_base {
public:
  client(const boost::property_tree::ptree& config)
    : client_base(network::get_io_service(), config)
    , p_(config) {}

  virtual ~client() {
    LOG_INFO("~client: close");
  }

  virtual void process(data_buffer_type::const_iterator begin,
                       data_buffer_type::const_iterator end) {
//     LOG_INFO(str(boost::format("client::process: h='%s', length=%d")
//                  % get_header()
//                  % std::distance(begin,end)));    
    processor::service_base::sptr sp(service_net::make(get_header(), get_directory()));
    p_.process(sp, begin, end);
  }

  virtual void directory_update(const broadcaster_directory& new_directory) {
    // NOP
  }

protected:
private:
  PROCESSOR p_;
} ;

#endif // _CLIENT_HPP_cm121230_
