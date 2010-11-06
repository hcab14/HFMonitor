// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_ACTION_AVERAGE_DENSITY_HPP_cm101026_
#define _FFT_ACTION_AVERAGE_DENSITY_HPP_cm101026_

#include <iostream>
#include <string>
#include <stdexcept>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "Spectrum.hpp"
#include "FFTResultSpectrumPowerInInterval.hpp"
#include "FFTResultCalibration.hpp"
#include "FFTProxy.hpp"
#include "FFTActionSpectrumInterval.hpp"

namespace Action {
  class AverageDensity : public SpectrumInterval {
  public:
    AverageDensity(const boost::property_tree::ptree& config)
      : SpectrumInterval(config)
      , fReference_(config.get<double>("fRef"))
      , bandwidth_(config.get<double>("bandwidth")) {
      name_ = "FindPeak";
    }
    
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PowerSpectrum& ps) {
      std::cout << "AverageDensity::perform " << std::endl;
      try {
        Result::SpectrumPowerInInterval::Handle 
          spp((useCalibration())
              ? boost::make_shared<Result::CalibratedSpectrumPowerInInterval>
              (fReference_, bandwidth_, boost::dynamic_pointer_cast<Result::Calibration>
               (p.getResult(calibrationKey())))
              : boost::make_shared<Result::SpectrumPowerInInterval>(fReference_, bandwidth_));
        if (spp->proc(ps))
          p.putResult(resultKey(), spp);
        if (plotSpectrum())
          p.putResult(resultKey()+"_plot",
                      boost::make_shared<Result::PowerSpectrumLine>(p.getApproxPTime(), ps));
      } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
      }
    }
  private:
    const double fReference_;          // nominal frequency / Hz
    const double bandwidth_;           // bandwidth / Hz
  } ;
} // namespace Action

#endif // _FFT_ACTION_AVERAGE_DENSITY_HPP_cm101026_
