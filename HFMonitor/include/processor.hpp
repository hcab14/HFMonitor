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
#ifndef _PROCESSOR_HPP_cm121221_
#define _PROCESSOR_HPP_cm121221_

#include <complex>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>
#include <boost/functional/factory.hpp>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>

#include "processor/service.hpp"

namespace processor {
  class base : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<base> sptr;

    enum {
      max_buffer_size = 1024*1024 // 1MB
    } ;
    typedef boost::array<char, max_buffer_size> data_buffer_type;
    
    base(const boost::property_tree::ptree&) {}
    virtual ~base() {};

    virtual void process(service_base::sptr, 
                         data_buffer_type::const_iterator,
			 data_buffer_type::const_iterator) {
      throw std::runtime_error("not implemented");
    }
  } ;

  class registry {
  public:
    typedef boost::function<base::sptr(const boost::property_tree::ptree&)> a_factory;
    typedef std::map<std::string, a_factory> map_type;

    registry() {}

    static base::sptr make(std::string key, const boost::property_tree::ptree& config) {
      map_type::iterator i(map().find(key));
      if (i == map().end())
        return base::sptr();
      return i->second(config);
    }

    template<typename T>
    static void reg(std::string key) {
      map()[key] = boost::factory<typename T::sptr>();
    }

    static map_type& map() { return map_; }
    static map_type map_;
  private:
  } ;

  class base_iq : public base {
  public:
    typedef boost::shared_ptr<base_iq> sptr;
    typedef std::vector<std::complex<double> > const_iterator;

    base_iq(const boost::property_tree::ptree& config)
      : base(config) {}

    virtual ~base_iq() {};
    virtual void process_iq(service_iq::sptr,
                            const_iterator,
                            const_iterator) {
      throw std::runtime_error("not implemented");
    }
  } ;

  class processor_txt : public base {
  public:
    typedef boost::shared_ptr<processor_txt> sptr;
    virtual ~processor_txt() {};
  } ;

  class processor_png : public base {
  public:
    typedef boost::shared_ptr<processor_png> sptr;
    virtual ~processor_png() {};
  } ;

}
#endif // _PROCESSOR_HPP_cm121221_
