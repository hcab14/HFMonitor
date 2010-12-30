// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_ACTION_FIND_PEAK_HPP_cm101026_
#define _FFT_ACTION_FIND_PEAK_HPP_cm101026_

#include <iostream>
#include <string>
#include <stdexcept>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "Spectrum.hpp"
#include "FFTResultSpectrumPeak.hpp"
#include "FFTResultCalibration.hpp"
#include "FFTResultCalibratedSpectrumPeak.hpp"
#include "FFTResultPowerSpectrumLine.hpp"
#include "FFTProxy.hpp"
#include "FFTActionSpectrumInterval.hpp"

namespace Action {
  class FindPeak : public SpectrumInterval {
  public:
    FindPeak(const boost::property_tree::ptree& config)
      : SpectrumInterval(config)
      , fReference_(config.get<double>("<xmlattr>.fRef_Hz"))
      , minRatio_(config.get<double>("<xmlattr>.minRatio")) {
      name_ = "FindPeak";
    }
    virtual ~FindPeak() {}
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PowerSpectrum& ps) {
      try {
        const Proxy::Base::ptime t(p.getApproxPTime());
        Result::SpectrumPeak::Handle 
          spp((useCalibration())
              ? boost::make_shared<Result::CalibratedSpectrumPeak>
              (t, fReference_, boost::dynamic_pointer_cast<Result::Calibration>(p.getResult(calibrationKey())))
              : boost::make_shared<Result::SpectrumPeak>(t, fReference_));
        if (spp->findPeak(p, s, ps, minRatio_))
          p.putResult(resultKey(), spp);
        if (plotSpectrum())
          p.putResult(resultKey()+"_plot",
                      boost::make_shared<Result::PowerSpectrumLine>(t, ps));
      } catch (const std::runtime_error& e) {
        LOG_WARNING(e.what());
      }
    }
  private:
    const double fReference_;          // nominal frequency / Hz
    const double minRatio_;            // min. ratio peak/background
  } ;
} // namespace Action

#endif // _FFT_ACTION_FIND_PEAK_HPP_cm101026_
