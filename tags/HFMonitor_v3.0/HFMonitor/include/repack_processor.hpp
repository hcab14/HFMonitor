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
#ifndef _REPACK_PROCESSOR_HPP_cm100818_
#define _REPACK_PROCESSOR_HPP_cm100818_

#include <iostream>
#include <vector>
#include <complex>
#include <boost/integer.hpp>
#include <boost/property_tree/ptree.hpp>

#include "logging.hpp"
#include "network.hpp"
#include "processor.hpp"
#include "repack_buffer.hpp"

template<typename PROCESSOR>
class repack_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<repack_processor> sptr;

  typedef processor::base_iq::complex_type complex_type;
  typedef processor::base_iq::const_iterator const_iterator;

  typedef repack_buffer<complex_type> repack_buffer_type;

  repack_processor(boost::asio::io_service& service,
                   const boost::property_tree::ptree& config)
    : base_iq(config)
    , p_(service, config)
    , bufferLengthSec_(config.get<double>("Repack.<xmlattr>.bufferLength_sec"))
    , overlap_(   0.01*config.get<double>("Repack.<xmlattr>.overlap_percent"))
    , repack_buffer_(1024, overlap_) {}

  repack_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , p_(config)
    , bufferLengthSec_(config.get<double>("Repack.<xmlattr>.bufferLength_sec"))
    , overlap_(   0.01*config.get<double>("Repack.<xmlattr>.overlap_percent"))
    , repack_buffer_(1024, overlap_) {}

  virtual ~repack_processor() {}

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    if (!sp_)
      repack_buffer_.resize(size_t(0.5 + sp->sample_rate_Hz()*bufferLengthSec_));

    // save the service sptr: it is used in process_samples for constructing a service_repack sptr
    sp_ = sp;
    repack_buffer_.insert(this, i0, i1);
  }
  
// protected:
  class service_repack : public service {
  public:
    typedef boost::shared_ptr<service_repack> sptr;
    virtual ~service_repack() {}

    static sptr make(service::sptr sp, boost::uint64_t counter) {
      return sptr(new service_repack(sp, counter));
    }
    virtual std::string     id()                  const { return sp_->id(); }
    virtual ptime           approx_ptime()        const { return t_; }
    virtual boost::uint16_t stream_number()       const { return sp_->stream_number(); }
    virtual std::string     stream_name()         const { return sp_->stream_name(); }
    virtual boost::uint32_t sample_rate_Hz()      const { return sp_->sample_rate_Hz(); }
    virtual double          center_frequency_Hz() const { return sp_->center_frequency_Hz(); }
    virtual float           offset_ppb()          const { return sp_->offset_ppb(); }
    virtual float           offset_ppb_rms()      const { return sp_->offset_ppb_rms(); }

    virtual void put_result(processor::result_base::sptr rp) {}
    virtual processor::result_base::sptr get_result(std::string name) const { return processor::result_base::sptr();  }

  protected:
  private:
    typedef boost::posix_time::time_duration time_duration;
    // counter is the distance (in samples) from sp_->approx_ptime()
    service_repack(service::sptr sp, boost::uint64_t counter)
      : sp_(sp)
      , t_(sp->approx_ptime() +
           time_duration(0,0,0,
                         counter*time_duration::ticks_per_second()/sp_->sample_rate_Hz())) {}
    const service::sptr sp_;
    const ptime         t_;
  } ;

public:
  // called from repack_buffer_type::insert
  //   * counter is the distance (in samples) from sp_->approx_ptime()
  void process_samples(const_iterator beg, const_iterator end, boost::uint64_t counter) { 
    typename service_repack::sptr sp_repack(service_repack::make(sp_, counter));
    p_.process_iq(sp_repack, beg, end);
  }

private:
  PROCESSOR          p_;
  const double       bufferLengthSec_;
  const double       overlap_;
  repack_buffer_type repack_buffer_;
  service::sptr      sp_;
} ;

#endif // _REPACK_PROCESSOR_HPP_cm100818_

