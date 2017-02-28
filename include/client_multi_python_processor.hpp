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

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "network.hpp"
#include "network/client/client_multi_base.hpp"

#include <boost/python.hpp>

class client_multi_python_processor : public network::client::client_multi_base {
public:
  typedef boost::shared_ptr<client_multi_python_processor> sptr;

  client_multi_python_processor(boost::asio::io_service& io_service,
                                const boost::property_tree::ptree& config)
    : network::client::client_multi_base(io_service, config)
    , python_pkg_  (config.get<std::string>("ClientMulti.Python.<xmlattr>.pkg"))
    , python_file_ (config.get<std::string>("ClientMulti.Python.<xmlattr>.file"))
    , python_fcn_  (config.get<std::string>("ClientMulti.Python.<xmlattr>.fcn")) {
    PyGILState_STATE gstate = PyGILState_Ensure();
    obj_pkg_      = boost::python::import(python_pkg_.c_str());
    obj_datetime_ = boost::python::import("datetime").attr("datetime");
    PyGILState_Release(gstate);
  }
  
  virtual ~client_multi_python_processor() {}

  virtual void proc_data_per_interval(const name_time_data_type& d) {
    PyGILState_STATE gstate = PyGILState_Ensure();
    boost::python::dict data;
    BOOST_FOREACH(const name_time_data_type::value_type& v, d) {
      boost::python::list list;
      BOOST_FOREACH(const time_data_type::value_type& u, v.second) {
        boost::python::list list_data;
        list_data.append(ptime_to_obj(u.first));
        const std::string sdata(&u.second.data()[0], u.second.data().size());
        add_data(list_data, sdata);
        list.append(boost::python::tuple(list_data));
      }
      data[boost::python::make_tuple(v.first.first, v.first.second)] = list;
    }
    // call python method
    obj_pkg_.attr(python_file_.c_str()).attr(python_fcn_.c_str())(data);
    PyGILState_Release(gstate);
  }

protected:
  boost::python::list add_data(boost::python::list list, std::string sdata) const {
    boost::algorithm::trim(sdata);
    std::vector<std::string> sv;
    boost::algorithm::split(sv, sdata, boost::algorithm::is_any_of(" "), boost::algorithm::token_compress_on);
    BOOST_FOREACH(std::string &s, sv) {
      try {
        list.append(boost::python::object(boost::lexical_cast<double>(s)));
      } catch (const boost::bad_lexical_cast &) {
        list.append(boost::python::object(s.c_str()));
      }
    }
    return list;
  }
  boost::python::object ptime_to_obj(const ptime& t) const {
    const boost::gregorian::date&             d = t.date();
    const boost::posix_time::time_duration& tod = t.time_of_day();
    return obj_datetime_(int(d.year()),
                         int(d.month()),
                         int(d.day()),
                         int(tod.hours()),
                         int(tod.minutes()),
                         int(tod.seconds()),
                         int(tod.fractional_seconds() * 1e6/boost::posix_time::time_duration::ticks_per_second()));
  }
private:
  std::string python_pkg_;
  std::string python_file_;
  std::string python_fcn_;
  boost::python::object obj_pkg_;
  boost::python::object obj_datetime_;
} ;

#endif // CLIENT_MULTI_PYTHON_PROCESSOR_HPP_cm20170228
