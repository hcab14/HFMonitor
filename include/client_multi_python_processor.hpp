// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2017 Christoph Mayer
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

#ifndef CLIENT_MULTI_PYTHON_PROCESSOR_HPP_cm20170228
#define CLIENT_MULTI_PYTHON_PROCESSOR_HPP_cm20170228

#include <boost/property_tree/xml_parser.hpp>\

#include "network.hpp"
#include "network/client/client_multi_base.hpp"

#include <boost/python.hpp>

class client_multi_python_processor : public network::client::client_multi_base {
public:
  typedef boost::shared_ptr<client_multi_python_processor> sptr;

  client_multi_python_processor(boost::asio::io_service& io_service,
                                const boost::property_tree::ptree& config)
    : network::client::client_multi_base(io_service, config)
    , obj_pkg_     (boost::python::import("test_package")) // TBD: via configuration
    , obj_datetime_(boost::python::import("datetime"))
  {}

  virtual ~client_multi_python_processor() {}

  virtual void proc_data_per_interval(const name_time_data_type& d) {
    boost::python::dict data;
    BOOST_FOREACH(const name_time_data_type::value_type& v, d) {
      boost::python::list list;
      BOOST_FOREACH(const time_data_type::value_type& u, v.second) {
        list.append(boost::python::make_tuple(ptime_to_obj(u.first),
                                              std::string(u.second.data().begin(),
                                                          u.second.data().end())));
      }
      data[v.first] = list;
    }
    // call python method
  }

protected:
  boost::python::object ptime_to_obj(const ptime& t) const {
    const boost::gregorian::date&             d = t.date();
    const boost::posix_time::time_duration& tod = t.time_of_day();
    return obj_datetime_.attr("datetime")(d.year(),
                                          d.month(),
                                          d.day(),
                                          tod.hours(),
                                          tod.minutes(),
                                          tod.seconds(),
                                          tod.fractional_seconds() * 1e6/boost::posix_time::time_duration::ticks_per_second());
  }
private:
  boost::python::object obj_pkg_;
  boost::python::object obj_datetime_;
} ;

#endif // CLIENT_MULTI_PYTHON_PROCESSOR_HPP_cm20170228
