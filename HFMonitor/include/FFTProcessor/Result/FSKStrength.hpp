// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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
#ifndef _FFT_RESULT_FSK_STRENGTH_HPP_cm101118_
#define _FFT_RESULT_FSK_STRENGTH_HPP_cm101118_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>

#include "Spectrum.hpp"
#include "FFTProcessor/Result.hpp"

namespace Result {
  class FFTResultFSKStrength : public Base {
  public:
    typedef boost::shared_ptr<FFTResultFSKStrength> sptr;
    typedef boost::shared_ptr<FFTResultFSKStrength> Handle;
    typedef frequency_vector<double> PowerSpectrum;
    FFTResultFSKStrength(ptime time,
                         Result::SpectrumPeak::Handle peakRef,
                         Result::SpectrumPeak::Handle peakShift,
                         Result::SpectrumPowerInInterval::Handle noise)
      : Base("FFTResultFSKStrength", time)
      , peakRef_(peakRef) 
      , peakShift_(peakShift) 
      , noise_(noise)
      , strength_dBm_(20.*std::log10(  std::pow(10., peakRef_->strength()  /20.)
                                     + std::pow(10., peakShift_->strength()/20.)))
      , ratio_dB_(strength_dBm_ - noise_->strength()) {}

    virtual ~FFTResultFSKStrength() {}
    virtual std::string toString() const { 
      std::stringstream ss; 
      ss << Base::toString() 
         << " fReference="     << peakRef_->fReference()
         << " fShift="         << peakShift_->fReference()
         << " fMeasuredRef="   << peakRef_->fMeasured()
         << " fMeasuredShift=" << peakShift_->fMeasured()
         << " strength="       << strength_dBm_
         << " strengthRef="    << peakRef_->strength()
         << " strengthShift="  << peakShift_->strength()
         << " noise="          << noise_->strength()
         << " ratio="          << ratio_dB_;
      return ss.str();
    }
    virtual std::ostream& dump_header(std::ostream& os) const {      
      Base::dump_header(os);
      return os
        << "fReference_Hz fShift_Hz fMeasuredRef_Hz fMeasuredShift_Hz "
        << "strength_dBm strengthRef_dBm strengthShift_dBm S/N_dB ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      Base::dump_data(os);
      return os
        << boost::format("%12.3f") % peakRef_->fReference() << " "
        << boost::format("%12.3f") % peakShift_->fReference() << " "
        << boost::format("%12.3f") % peakRef_->fMeasured() << " "
        << boost::format("%12.3f") % peakShift_->fMeasured() << " "
        << boost::format("%7.2f")  % strength_dBm_ << " "
        << boost::format("%7.2f")  % peakRef_->strength() << " "
        << boost::format("%7.2f")  % peakShift_->strength() << " "
        << boost::format("%7.2f")  % ratio_dB_ << " ";
    }

  private:
    const Result::SpectrumPeak::Handle peakRef_;
    const Result::SpectrumPeak::Handle peakShift_;
    const Result::SpectrumPowerInInterval::Handle noise_;
    const double strength_dBm_;
    const double ratio_dB_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_FSK_STRENGTH_HPP_cm101118_
