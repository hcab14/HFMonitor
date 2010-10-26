// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _RAW_2_IQ_ADAPTER_HPP_cm100729_
#define _RAW_2_IQ_ADAPTER_HPP_cm100729_

#include <complex>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/integer.hpp>
#include "protocol.hpp"

template<typename PROCESSOR>
class Raw2IQAdapter {
private:
  typedef union {
    struct __attribute__((__packed__)) {
      boost::int32_t i;
      boost::int32_t q;
    } iq;
    struct __attribute__((__packed__)) {
      boost::uint8_t i1,i2,i3,i4;
      boost::uint8_t q1,q2,q3,q4;
    } ;
  } IQSample;

public:
  Raw2IQAdapter(const boost::property_tree::ptree& config)
    : p_(config) {}
  ~Raw2IQAdapter() {}

  void procRaw(const Header& header, 
               std::vector<char>::const_iterator i0,
               std::vector<char>::const_iterator i1) {
//    std::cout << "Raw2IQAdapter::procRaw " << header << " data.size()= " << data.size() << std::endl;
    const size_t size(std::distance(i0, i1));
    const double norm(1. / double(1<<23));
    std::vector<std::complex<double> > iqs;
    if (size == 6 * header.numberOfSamples()) {
      for (std::vector<char>::const_iterator i(i0); i!=i1;) {
        IQSample s;
        s.i1 = 0; s.i2 = *i++; s.i3 = *i++; s.i4 = *i++;
        s.q1 = 0; s.q2 = *i++; s.q3 = *i++; s.q4 = *i++;
        const std::complex<double> cs(s.iq.q*norm,
                                      s.iq.i*norm);
        // std::cout << cs << std::endl;
        iqs.push_back(cs);
      }
    } else {
      throw std::runtime_error("Raw2IQAdapter::procRaw size != 6 * header.numberOfSamples())");
    }
    p_.procIQ(header, iqs.begin(), iqs.end());
  }
protected:
private:
  PROCESSOR p_;
} ;

#endif // _RAW_2_IQ_ADAPTER_HPP_cm100729_

