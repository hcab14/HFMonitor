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
#ifndef _FFT_RESULT_CALIBRATED_SPECTRUM_PEAK_HPP_cm101026_
#define _FFT_RESULT_CALIBRATED_SPECTRUM_PEAK_HPP_cm101026_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>

#include "FFTProcessor/Result/SpectrumPeak.hpp"
#include "FFTProcessor/Result/Calibration.hpp"

namespace Result {
  class CalibratedSpectrumPeak : public SpectrumPeak {
  public:
    typedef boost::shared_ptr<SpectrumPeak> sptr;
    typedef boost::shared_ptr<SpectrumPeak> Handle;
    CalibratedSpectrumPeak(ptime time,
                           double fReference,
                           Calibration::Handle calibrationHandle)
      : SpectrumPeak(time, fReference)
      , calibrationHandle_(calibrationHandle) {
      processor::result_base::set_name("CalibratedSpectrumPeak");
    }
    virtual ~CalibratedSpectrumPeak() {}

    virtual std::string toString() const { 
      std::stringstream ss; 
      ss << SpectrumPeak::toString()
         << " diff=" << fMeasured()-fReference();
      return ss.str();
    }

    virtual std::pair<double, double> cal(double f) const {
      return calibrationHandle_->uncal2cal(f);
    }

    virtual std::ostream& dump_header(std::ostream& os) const {      
      return SpectrumPeak::dump_header(os) << "diff_Hz ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      return SpectrumPeak::dump_data(os) 
        << boost::format("%8.3f")  % (fMeasured()-fReference()) << " ";
    }

  private:
    Calibration::Handle calibrationHandle_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_CALIBRATED_SPECTRUM_PEAK_HPP_cm101026_
