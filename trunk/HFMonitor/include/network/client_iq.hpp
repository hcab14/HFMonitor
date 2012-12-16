// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _CLIENT_IQ_HPP_cm121126_
#define _CLIENT_IQ_HPP_cm121126_

#include "processor/service.hpp"
#include "network/protocol.hpp"
#include "network/client.hpp"

namespace {
  class service_net_iq : public processor::service_iq {
  public:
    typedef boost::shared_ptr<service_net_iq> sptr;
    virtual ~service_net_iq() {}

    static sptr make(const header& h, const iq_info& hiq) {
      return sptr(new service_net_iq(h, hiq)); 
    }

    virtual std::string     id()                  const { return header_.id(); }
    virtual ptime           approx_ptime()        const { return header_.approx_ptime(); }
    virtual boost::uint16_t stream_number()       const { return header_.stream_number(); }
    virtual boost::uint32_t sample_rate_Hz()      const { return iq_info_.sample_rate_Hz(); }
    virtual double          center_frequency_Hz() const { return iq_info_.center_frequency_Hz(); }
    virtual float           offset_ppb()          const { return iq_info_.offset_ppb(); }
    virtual float           offset_ppb_rms()      const { return iq_info_.offset_ppb_rms(); }

    virtual ptime update_ptime(time_duration dt) { return header_.update_ptime(dt); }
  protected:
  private:
    service_net_iq(const header& h, const iq_info& hiq)
      : header_(h)
      , iq_info_(hiq) {}

    header  header_;
    iq_info iq_info_;
  } ;
} // anonymous namespace

// client for streams of I/Q samples
template<typename PROCESSOR>
class client_iq : public client {
  // for integer I/Q samples
  typedef union {
    struct __attribute__((__packed__)) {
      boost::int32_t i;
      boost::int32_t q;
    } iq;
    struct __attribute__((__packed__)) {
      boost::uint8_t i1,i2,i3,i4;
      boost::uint8_t q1,q2,q3,q4;
    } samples;
  } iq_sample;

public:
  client_iq(boost::asio::io_service&           io_service,
	    const boost::property_tree::ptree& config)
    : client(io_service, config)
    , p_(config) {}

  virtual ~client_iq() {}

  virtual void process(data_buffer_type::const_iterator begin,
                       data_buffer_type::const_iterator end) {
    iq_info header_iq;
    const header& h(get_header());
    // std::cout << "sizeof(iq_info)= " << sizeof(iq_info) << std::endl;
    bcopy(begin, &header_iq, sizeof(iq_info));
    begin += sizeof(iq_info);
    // std::cout << "process: " << h << " " << header_iq << std::endl;
    std::vector<std::complex<double> > iqs;
    if (header_iq.sample_type() == 'I' && header_iq.bytes_per_sample() ==3) {
      const double norm(1./static_cast<double>(1L << 31));
      for (data_buffer_type::const_iterator i(begin); i!=end;) {
        iq_sample s;
        s.samples.i1 = 0; s.samples.i2 = *i++; s.samples.i3 = *i++; s.samples.i4 = *i++;
        s.samples.q1 = 0; s.samples.q2 = *i++; s.samples.q3 = *i++; s.samples.q4 = *i++;
        const std::complex<double> cs(s.iq.q*norm,
                                      s.iq.i*norm);
        iqs.push_back(cs);
      }
      // make up service object
      service_net_iq::sptr sp(service_net_iq::make(h, header_iq));

      // call processor
      p_.process_iq(sp, iqs.begin(), iqs.end());
    } else {
      // complain
    }
  }
protected:

private:
  PROCESSOR p_;
} ;


#endif // _CLIENT_IQ_HPP_cm121126_ 
