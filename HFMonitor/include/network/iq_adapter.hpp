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
#ifndef _IQ_ADAPTER_HPP_cm121126_
#define _IQ_ADAPTER_HPP_cm121126_

#include "processor.hpp"
#include "network/protocol.hpp"
#include "network/client/service_net.hpp"
#include "wave/reader.hpp"

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
  typedef boost::shared_ptr<iq_adapter > sptr;

  iq_adapter(const boost::property_tree::ptree& config)
    : base(config)
    , p_(config) {}
  
  virtual ~iq_adapter() {}
  
  virtual void process(service::sptr sp,
                       const_iterator begin,
                       const_iterator end) {
    const bool is_iq (std::string(sp->id(), 0, 2) == "IQ");
    const bool is_wav(std::string(sp->id(), 0, 3) == "WAV");
    if (not is_iq && not is_wav)
      return;

    iq_info header_iq;
    bcopy(begin, &header_iq, sizeof(iq_info));
    begin += sizeof(iq_info);

    std::vector<std::complex<double> > iqs;
    if (header_iq.sample_type() == 'I' && header_iq.bytes_per_sample() ==3) {
      if (is_iq) {
        const double norm(1./static_cast<double>(1L << 31));
        for (const_iterator i(begin); i!=end;) {
          iq_sample s;
          s.samples.i1 = 0; s.samples.i2 = *i++; s.samples.i3 = *i++; s.samples.i4 = *i++;
          s.samples.q1 = 0; s.samples.q2 = *i++; s.samples.q3 = *i++; s.samples.q4 = *i++;
          const std::complex<double> cs(s.iq.q*norm,
                                        s.iq.i*norm);
          iqs.push_back(cs);
        }
      } else if (is_wav) {
        const size_t bytes_per_sample(header_iq.bytes_per_sample());
        assert((std::distance(begin, end) % 2*bytes_per_sample) == 0);
        std::istringstream iss_iq;
        std::cout << "is_wav bytes_per_sample= " << bytes_per_sample 
                  << " n= " << std::distance(begin, end) << std::endl;
        for (const_iterator i(begin); i!=end;) {
          const std::string str_iq(i, i+2*bytes_per_sample); i+=2*bytes_per_sample;
          iss_iq.str(str_iq);
          const double xi(wave::detail::read_real_sample(iss_iq, 8*bytes_per_sample)); // real
          const double xq(wave::detail::read_real_sample(iss_iq, 8*bytes_per_sample)); // imag
          iqs.push_back(std::complex<double>(xi,xq));
        }
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
