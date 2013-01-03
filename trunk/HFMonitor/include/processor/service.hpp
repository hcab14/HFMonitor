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
#ifndef _SERVICE_HPP_cm110729_
#define _SERVICE_HPP_cm110729_

#include <string>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace processor {
  // base class for source-specific parameters to the service object
  class specific_param_base {
  public:
    typedef boost::shared_ptr<specific_param_base> sptr;
    virtual ~specific_param_base() {}

    virtual std::string to_string() const {
      return "specific_param_base"; 
    }
  } ;
  
  class service_base : private boost::noncopyable {
  public:
    typedef boost::posix_time::ptime ptime;
    typedef boost::shared_ptr<service_base> sptr;
    typedef boost::posix_time::time_duration time_duration;
    
    service_base() {}
    virtual ~service_base() {}
    
    virtual std::string     id() const = 0;
    virtual ptime           approx_ptime() const = 0;
    virtual boost::uint16_t stream_number() const = 0;

    virtual ptime update_ptime(time_duration) = 0;
  protected:
  private:
  } ;

  class service_iq : public service_base {
  public:
    typedef boost::shared_ptr<service_iq> sptr;
    
    service_iq() {}
    virtual ~service_iq() {}
    
    virtual boost::uint32_t sample_rate_Hz()      const = 0;
    virtual double          center_frequency_Hz() const = 0;
    virtual float           offset_ppb()          const = 0;
    virtual float           offset_ppb_rms()      const = 0;
  } ;

  // // processor service object 
  // class service_base : private boost::noncopyable {
  // public:
  //   typedef boost::posix_time::ptime ptime;
  //   typedef boost::shared_ptr<service_base> sptr;

  //   service_base(std::string source_name)
  //     : source_name_(source_name) {}
  //   virtual ~service_base() {}

  //   // name of iq data source
  //   virtual std::string source_name() const { return source_name_; }

  //   virtual double center_freq_hz() const = 0;
  //   virtual double sample_rate_hz() const = 0;

  //   // approx. time at start of samples
  //   virtual ptime approx_ptime() const = 0;

  //   // param specific to the source
  //   virtual specific_param_base::sptr specific_param() const = 0;

  //   // return a service object with samplerate changed by factor num/denum
  //   virtual sptr resample(size_t num, size_t denum) const = 0;

  //   // return a service object with a shifted center frequency
  //   virtual sptr freq_shift(double freq_shift_hz) const = 0;

  // protected:
  // private:
  //   const std::string source_name_;
  // } ;
} // namespace processor
#endif // _SERVICE_HPP_cm110729_
