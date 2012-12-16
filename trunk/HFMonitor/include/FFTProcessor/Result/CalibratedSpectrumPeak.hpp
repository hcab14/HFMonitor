// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
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
    typedef boost::shared_ptr<SpectrumPeak> Handle;
    CalibratedSpectrumPeak(ptime time,
                           double fReference,
                           Calibration::Handle calibrationHandle)
      : SpectrumPeak(time, fReference)
      , calibrationHandle_(calibrationHandle) {
      name_= "CalibratedSpectrumPeak";
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

    virtual std::ostream& dumpHeader(std::ostream& os) const {      
      return SpectrumPeak::dumpHeader(os) << "diff_Hz ";
    }
    virtual std::ostream& dumpData(std::ostream& os) const {
      return SpectrumPeak::dumpData(os) 
        << boost::format("%8.3f")  % (fMeasured()-fReference()) << " ";
    }

  private:
    Calibration::Handle calibrationHandle_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_CALIBRATED_SPECTRUM_PEAK_HPP_cm101026_
