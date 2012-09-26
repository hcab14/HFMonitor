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
#ifndef _SERVICE_GENERIC_IQ_HPP_cm110729_
#define _SERVICE_GENERIC_IQ_HPP_cm110729_

#include "service.hpp"

namespace processor {
  class service_generic_iq : public service_base {
  public:
    typedef boost::shared_ptr<service_generic_iq> sptr;

    service_generic_iq(double center_freq_hz,
                       double sample_rate_hz,
                       ptime  approx_ptime)
      : service_base("generic I/Q")
      , center_freq_hz_(center_freq_hz)
      , sample_rate_hz_(sample_rate_hz)
      , approx_ptime_(approx_ptime) {
      std::cerr << "s_g_iq " << center_freq_hz_ << " "
                << sample_rate_hz_ << " " << approx_ptime_ << std::endl;
    }

    virtual ~service_generic_iq() {}

    virtual double center_freq_hz() const { return center_freq_hz_; }
    virtual double sample_rate_hz() const { return sample_rate_hz_; }

    // approx. time at start of samples
    virtual ptime approx_ptime() const { return approx_ptime_; }

    // param specific to the source
    virtual specific_param_base::sptr specific_param() const {
      return specific_param_base::sptr(new specific_param_base);
    }

    // return a service object with samplerate changed by factor num/denum
    virtual service_base::sptr resample(size_t num, size_t denum) const {
      return sptr(new service_generic_iq(center_freq_hz_,
                                         sample_rate_hz_*num/denum,
                                         approx_ptime_));
    }

    // return a service object with a shifted center frequency
    virtual service_base::sptr freq_shift(double freq_shift_hz) const {
      return sptr(new service_generic_iq(center_freq_hz_+freq_shift_hz,
                                         sample_rate_hz_,
                                         approx_ptime_));
    }

  protected:
  private:
    double center_freq_hz_;
    double sample_rate_hz_;
    ptime  approx_ptime_;
  } ;
} // namespace processor
#endif // _SERVICE_GENERIC_IQ_HPP_cm110729_
