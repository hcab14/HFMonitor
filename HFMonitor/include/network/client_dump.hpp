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
#ifndef _CLIENT_DUMP_HPP_cm121217_
#define _CLIENT_DUMP_HPP_cm121217_

#include "processor/service.hpp"
#include "network/protocol.hpp"
#include "network/client.hpp"

namespace {

// client for streams of I/Q samples
template<typename PROCESSOR>
class client_dump : public client {
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
  client_dump(boost::asio::io_service&        io_service,
	    const boost::property_tree::ptree& config)
    : client(io_service, config)
    , p_(config) {}

  client_dump(const boost::property_tree::ptree& config)
    : client(config)
    , p_(config) {}

  virtual ~client_dump() {}

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

#endif // _CLIENT_DUMP_HPP_cm121126_ 
