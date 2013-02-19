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
#ifndef _SERVICE_HPP_cm110729_
#define _SERVICE_HPP_cm110729_

#include <string>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "processor/result.hpp"

namespace processor {
  
  class service_base : private boost::noncopyable {
  public:
    typedef boost::posix_time::ptime ptime;
    typedef boost::shared_ptr<service_base> sptr;
    typedef boost::posix_time::time_duration time_duration;
    
    virtual ~service_base() {}
    
    virtual std::string     id() const = 0;
    virtual ptime           approx_ptime() const = 0;
    virtual boost::uint16_t stream_number() const = 0;
    virtual std::string     stream_name() const = 0;

    virtual void              put_result(result_base::sptr ) {}
    virtual result_base::sptr get_result(std::string name) const {
      return result_base::sptr();
    }
  protected:
  private:
  } ;

  class service_iq : public service_base {
  public:
    typedef boost::shared_ptr<service_iq> sptr;
    
    virtual ~service_iq() {}
    
    virtual boost::uint32_t sample_rate_Hz()      const = 0;
    virtual double          center_frequency_Hz() const = 0;
    virtual float           offset_ppb()          const = 0;
    virtual float           offset_ppb_rms()      const = 0;
  } ;

} // namespace processor
#endif // _SERVICE_HPP_cm110729_
