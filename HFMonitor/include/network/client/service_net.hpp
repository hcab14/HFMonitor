// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _SERVICE_NET_HPP_cm130103_
#define _SERVICE_NET_HPP_cm130103_

#include <iostream>
#include <set>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include "logging.hpp"
#include "network/protocol.hpp"
#include "processor.hpp"

class service_net : public processor::service_base {
public:
  typedef boost::shared_ptr<service_net> sptr;
  virtual ~service_net() {}
  
  static sptr make(const header& h) {
    sptr result(new service_net(h));
    return result;
  }
  
  virtual std::string     id()            const { return header_.id(); }
  virtual ptime           approx_ptime()  const { return header_.approx_ptime(); }
  virtual boost::uint16_t stream_number() const { return header_.stream_number(); }
  
  virtual ptime update_ptime(time_duration dt) { return header_.update_ptime(dt); }
protected:
private:
  service_net(const header& h)
  : service_base()
  , header_(h) {}
  
  header  header_;
} ;

class service_net_iq : public processor::service_iq {
public:
  typedef boost::shared_ptr<service_net_iq> sptr;
  virtual ~service_net_iq() {}
  
  static sptr make(processor::service_base::sptr sp,
                   const iq_info& hiq) {
    sptr result(new service_net_iq(sp, hiq));
    return result;
  }
  
  virtual std::string     id()                  const { return sp_->id(); }
  virtual ptime           approx_ptime()        const { return sp_->approx_ptime(); }
  virtual boost::uint16_t stream_number()       const { return sp_->stream_number(); }
  virtual boost::uint32_t sample_rate_Hz()      const { return iq_info_.sample_rate_Hz(); }
  virtual double          center_frequency_Hz() const { return iq_info_.center_frequency_Hz(); }
  virtual float           offset_ppb()          const { return iq_info_.offset_ppb(); }
  virtual float           offset_ppb_rms()      const { return iq_info_.offset_ppb_rms(); }
  
  virtual ptime update_ptime(time_duration dt) { return sp_->update_ptime(dt); }
protected:
private:
  service_net_iq(processor::service_base::sptr sp,
                 const iq_info& hiq)
    : sp_(sp)
    , iq_info_(hiq) {}
  
  processor::base::service::sptr sp_;
  iq_info iq_info_;
} ;

#endif // _SERVICE_NET_HPP_cm130103_
