// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _SERVICE_PERSEUS_HPP_cm110729_
#define _SERVICE_PERSEUS_HPP_cm110729_

#include "service_generic_iq.hpp"

namespace processor {

  // source-specific parameters for data from the perseus SDR
  class perseus_param : public specific_param_base {
  public:
    typedef boost::shared_ptr<perseus_param> sptr;
    perseus_param(protocol::perseus::header header)
      : header_(header) {}
    virtual ~perseus_param() {}

    // TODO
    virtual std::string to_string() const {
      return "perseus_param_base";
    }
  private:
    const protocol::perseus::header header_;
  } ;

  // processor service object for perseus data
  class service_perseus : public service_base {
  public:
    typedef boost::posix_time::ptime ptime;
    typedef boost::shared_ptr<service_perseus> sptr;
    
    service_perseus(const protocol::perseus::header& header)
      : service_base("service_perseus")
      , header_(header) {}
    virtual ~service_perseus() {}

    virtual double center_freq_hz() const { return header_.ddcCenterFrequency(); }
    virtual double sample_rate_hz() const { return header_.sampleRate(); }

    // approx. time at start of samples
    virtual ptime approx_ptime() const { return header_.approxPTime(); }

    // param specific to the source
    virtual specific_param_base::sptr specific_param() const {
      return perseus_param::sptr(new perseus_param(header_));
    }

    // return a service object with samplerate changed by factor num/denum
    virtual service_base::sptr resample(size_t num, size_t denum) const {
      return sptr(new service_perseus(header_.resample(num, denum)));
    }  
    // return a service object with a shifted center frequency
    virtual service_base::sptr freq_shift(double freq_shift_hz) const {
      return sptr(new service_perseus(header_.freq_shift(freq_shift_hz)));
    }

  protected:
  private:
    const protocol::perseus::header header_;
  } ;  
} // namespace processor
#endif // _SERVICE_PERSEUS_HPP_cm110729_
