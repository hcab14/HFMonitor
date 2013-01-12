// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _IQ_ADAPTER_HPP_cm121126_
#define _IQ_ADAPTER_HPP_cm121126_

#include "processor.hpp"
#include "network/protocol.hpp"
#include "network/client/service_net.hpp"

// adapter
template<typename PROCESSOR>
class iq_adapter : public processor::base {
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

  iq_adapter(const boost::property_tree::ptree& config)
    : base(config)
    , p_(config) {}
  
  virtual ~iq_adapter() {}
  
  void process(service::sptr sp,
               const_iterator begin,
               const_iterator end) {
    if (std::string(sp->id(), 0, 2) != "IQ")
      return;

    iq_info header_iq;
    bcopy(begin, &header_iq, sizeof(iq_info));
    begin += sizeof(iq_info);

    std::vector<std::complex<double> > iqs;
    if (header_iq.sample_type() == 'I' && header_iq.bytes_per_sample() ==3) {
      const double norm(1./static_cast<double>(1L << 31));
      for (const_iterator i(begin); i!=end;) {
        iq_sample s;
        s.samples.i1 = 0; s.samples.i2 = *i++; s.samples.i3 = *i++; s.samples.i4 = *i++;
        s.samples.q1 = 0; s.samples.q2 = *i++; s.samples.q3 = *i++; s.samples.q4 = *i++;
        const std::complex<double> cs(s.iq.q*norm,
                                      s.iq.i*norm);
        iqs.push_back(cs);
      }

      // make up service object
      service_net_iq::sptr snp(service_net_iq::make(sp, header_iq));

      // call processor
      p_.process_iq(snp, iqs.begin(), iqs.end());
    } else {
      // complain
    }
  }
protected:

private:
  PROCESSOR p_;
} ;

#endif // _IQ_ADAPTER_HPP_cm121126_ 
