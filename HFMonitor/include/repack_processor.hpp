// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
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
#include "processor/IQBuffer.hpp"

template<typename PROCESSOR>
class repack_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<repack_processor> sptr;

  repack_processor(boost::asio::io_service&           service,
                   const boost::property_tree::ptree& config)
    : base_iq(config)
    , p_(service, config)
    , bufferLengthSec_(config.get<double>("Repack.<xmlattr>.bufferLength_sec"))
    , overlap_(   0.01*config.get<double>("Repack.<xmlattr>.overlap_percent"))
    , service_()
    , iqBuffer_(4, 0.0)
    , counter_(0) {}

  repack_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , p_(config)
    , bufferLengthSec_(config.get<double>("Repack.<xmlattr>.bufferLength_sec"))
    , overlap_(   0.01*config.get<double>("Repack.<xmlattr>.overlap_percent"))
    , service_()
    , iqBuffer_(4, 0.0)
    , counter_(0) {}

  ~repack_processor() {}

  boost::asio::io_service& get_service() { return service_; }

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    using namespace boost::posix_time;
    if (!service_)
      iqBuffer_.update(size_t(sp->sample_rate_Hz()*bufferLengthSec_), overlap_);

    const time_duration dt
      (0,0,0, counter_*time_duration::ticks_per_second()/sp->sample_rate_Hz());
    const ptime t(sp->approx_ptime()-dt);
    service_ = service_repack::make(sp, t);

    // if (!currentHeader_.hasEqualParameters(header)) 
    //   update(header);    

    // keep track of sample numbers
    for (; i0 != i1; ++i0) {
      ++counter_;
      iqBuffer_.insert(this, *i0);
    }
  }
  
  // called from IQBuffer::insert 
  void procIQ(std::vector<std::complex<double> >::const_iterator i0,
              std::vector<std::complex<double> >::const_iterator i1) {
    // std::cout << "counter= " << counter_ << " " << iqBuffer_.m() << std::endl;
    if (counter_ == iqBuffer_.m());
    p_.process_iq(service_, i0, i1);
    counter_ = 0;
  }

protected:
  class service_repack : public service {
  public:
    typedef boost::shared_ptr<service_repack> sptr;
    virtual ~service_repack() {}

    static sptr make(service::sptr sp, ptime t) {
      return sptr(new service_repack(sp, t));
    }
    virtual std::string     id()                  const { return sp_->id(); }
    virtual ptime           approx_ptime()        const { return t_; }
    virtual boost::uint16_t stream_number()       const { return sp_->stream_number(); }
    virtual std::string     stream_name()         const { return sp_->stream_name(); }
    virtual boost::uint32_t sample_rate_Hz()      const { return sp_->sample_rate_Hz(); }
    virtual double          center_frequency_Hz() const { return sp_->center_frequency_Hz(); }
    virtual float           offset_ppb()          const { return sp_->offset_ppb(); }
    virtual float           offset_ppb_rms()      const { return sp_->offset_ppb_rms(); }

  protected:
  private:
    service_repack(service::sptr sp, ptime t)
      : sp_(sp)
      , t_(t) {}
    const service::sptr sp_;
    const ptime         t_;
  } ;
private:
  PROCESSOR      p_;
  double         bufferLengthSec_;
  double         overlap_;
  service::sptr  service_;
  IQBuffer       iqBuffer_;
  boost::int64_t counter_;
} ;

#endif // _REPACK_PROCESSOR_HPP_cm100818_

