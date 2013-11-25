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
#ifndef _DEMOD_FSK_PROCESSOR_HPP_cm131117_
#define _DEMOD_FSK_PROCESSOR_HPP_cm131117_

#include <complex>
#include <iostream>
#include <sstream>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "demod/msk.hpp"
#include "filter/goertzel.hpp"
#include "filter/fir_filter.hpp"
#include "FFT.hpp"
#include "FFTProcessor/Filter.hpp"
#include "logging.hpp"
#include "processor.hpp"
#include "Spectrum.hpp"

class demod_fsk_processor : public processor::base_iq {
public:
  typedef boost::shared_ptr<demod_fsk_processor> sptr;
  typedef std::complex<double> complex_type;
  
  class result_bits : public processor::result_base {
  public:
    typedef boost::shared_ptr<result_bits> sptr;
    typedef std::vector<bool> bit_vector_type;
    typedef bit_vector_type::iterator iterator;
    typedef bit_vector_type::const_iterator const_iterator;

    virtual ~result_bits() {}
    static sptr make(std::string name,
                     ptime  t,
                     double fc_Hz,
                     double fm_Hz) {
      return sptr(new result_bits(name, t, fc_Hz, fm_Hz));
    }

    double fc_Hz()   const { return fc_Hz_; }
    double fm_Hz()   const { return fm_Hz_; }
    double quality() const { return quality_; }

    void  set_quality(double q) { quality_ = q; }

    size_t size() const { return bitvec_.size(); }
    const_iterator begin() const { return bitvec_.begin(); }
    const_iterator end() const { return bitvec_.end(); }

    void clear() { bitvec_.clear(); }
    void push_back(bool bit) { bitvec_.push_back(bit); }

    virtual std::ostream& dump_header(std::ostream& os) const {      
      return os
        << "# fc[Hz] = " << boost::format("%15.8f") % fc_Hz() << "\n"
        << "# fm[Hz] = " << boost::format("%7.3f")  % fm_Hz() << "\n"
        << "# Time_UTC quality[%] bits ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      os << boost::format("%3.0f") % (100.*quality()) << " ";
      for (const_iterator i(begin()); i!=end();) {
        unsigned char x(0);
        for (size_t j(0); j<4 && i!=end(); ++j, ++i)
          x |= (*i << (3-j));
        const char c(x<10 ? '0'+x : 'A'+x-10);
        os << c;
      }
      return os;
    }
    
  protected:
  private:
    result_bits(std::string name,
               ptime  t,
               double fc_Hz,
               double fm_Hz)
      : result_base(name, t)
      , fc_Hz_(fc_Hz)
      , fm_Hz_(fm_Hz)
      , quality_(0) {}
    
    const double    fc_Hz_;
    const double    fm_Hz_;
    double          quality_;
    bit_vector_type bitvec_;
  } ;
 
  typedef boost::posix_time::time_duration time_duration;

  typedef goertzel<std::complex<double> > goertzel_type;

  demod_fsk_processor(const boost::property_tree::ptree& config)
    : base_iq(config)
    , name_(config.get<std::string>("<xmlattr>.name"))
    , fc_Hz_(config.get<double>("<xmlattr>.fc_Hz"))
    , fs_Hz_(config.get<double>("<xmlattr>.fs_Hz"))
    , baud_(config.get<double>("<xmlattr>.baud"))
    , is_initialized_(false)
    , period_(1)
    , sample_counter_(0)
    , filter_(401, 0.025*baud_/200.) {}
  
  virtual ~demod_fsk_processor() {}

  double fc_Hz() const { return fc_Hz_; }
  double fs_Hz() const { return fs_Hz_; }
  double baud()  const { return baud_; }

  void process_iq(service::sptr sp,
                  const_iterator i0,
                  const_iterator i1) {
    // const size_t length(std::distance(i0, i1));

    const double offset_ppb(sp->offset_ppb());
    const double offset_Hz(fc_Hz()*offset_ppb*1e-9);
    if (not is_initialized_) {
      // measurements per bit
      period_ = size_t(0.5+sp->sample_rate_Hz()/baud()/10);

      const double f_shift0_Hz(fc_Hz() - sp->center_frequency_Hz() - offset_Hz);
      double f_shift_Hz(filter_.shift(f_shift0_Hz/sp->sample_rate_Hz())*sp->sample_rate_Hz());
      fc_Hz_ -= f_shift_Hz;

      // normalized mark frequency
      double k_mark ((fc_Hz() - fs_Hz() - sp->center_frequency_Hz()) / sp->sample_rate_Hz());
      k_mark  = round(k_mark/period_*sp->sample_rate_Hz())/sp->sample_rate_Hz()*period_;
      gf_mark_.set_parameter (k_mark);

      // normalized shift frequency
      double k_shift((fc_Hz() + fs_Hz() - sp->center_frequency_Hz()) / sp->sample_rate_Hz());
      k_shift = round(k_shift/period_*sp->sample_rate_Hz())/sp->sample_rate_Hz()*period_;
      gf_shift_.set_parameter(k_shift);

      std::cout << "k_mark,shift,period= " << k_mark << " " << k_shift << " " << period_ << " " 
                << sp->sample_rate_Hz() << " " 
                << sp->center_frequency_Hz() << std::endl;

      is_initialized_ = true;
    }

    for (const_iterator i(i0); i!=i1; ++i) {
      const complex_type sample(filter_.process(*i));
      std::cout << "S " << sample.real() <<  " " << sample.imag() << "\n";
      gf_mark_.update(sample);
      gf_shift_.update(sample);
      ++sample_counter_;
      if (sample_counter_ == period_) {
        std::cout << std::abs(gf_mark_.x()) << " " 
                  << std::abs(gf_shift_.x()) << "\n";
        sample_counter_ = 0;
        gf_mark_.reset();
        gf_shift_.reset();
      }
    }

  }

  // virtual void dump(processor::result_base::sptr) {
  //   // to be overwritten in a derived class
  // }

protected:
private:
  const std::string name_;
  double            fc_Hz_;     // center frequency
  const double      fs_Hz_;     // shift frequency
  const double      baud_;
  bool              is_initialized_;
  size_t            period_;
  size_t            sample_counter_;
  fir_filter        filter_;
  goertzel_type     gf_mark_;
  goertzel_type     gf_shift_;
  result_bits::sptr result_bits_; // accumulates bits
} ;

#endif // _DEMOD_FSK_PROCESSOR_HPP_cm131117_
