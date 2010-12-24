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
      : SpectrumInterval(addKeysToConfig(config,
                                         config.get<double>("<xmlattr>.fRef_Hz"),
                                         config.get<double>("<xmlattr>.bandwidth_Hz")))
      , fReference_(config.get<double>("<xmlattr>.fRef_Hz"))
      , bandwidth_ (config.get<double>("<xmlattr>.bandwidth_Hz")) {
      name_ = "FindPeak";
    }
    
    virtual void proc(Proxy::Base& p, 
                      const SpectrumBase& s,
                      const PowerSpectrum& ps) {
      LOG_INFO("AverageDensity::perform ");
      try {
        Result::SpectrumPowerInInterval::Handle 
          spp((useCalibration())
              ? boost::make_shared<Result::CalibratedSpectrumPowerInInterval>
              (fReference_, bandwidth_, s.normWindow(), boost::dynamic_pointer_cast<Result::Calibration>
               (p.getResult(calibrationKey())))
              : boost::make_shared<Result::SpectrumPowerInInterval>(fReference_, bandwidth_, s.normWindow()));
        if (spp->proc(ps))
          p.putResult(resultKey(), spp);
        if (plotSpectrum())
          p.putResult(resultKey()+"_plot",
                      boost::make_shared<Result::PowerSpectrumLine>(p.getApproxPTime(), ps));
      } catch (const std::runtime_error& e) {
        LOG_WARNING(e.what());
      }
    }
  private:
    // add keys "<xmlattr>.f{Min,Max}_Hz"
    static boost::property_tree::ptree addKeysToConfig(const boost::property_tree::ptree& config,
                                                       double fRef,
                                                       double bandwidth) {
      boost::property_tree::ptree result(config);
      result.put("<xmlattr>.fMin_Hz", boost::lexical_cast<std::string>(fRef-bandwidth));
      result.put("<xmlattr>.fMax_Hz", boost::lexical_cast<std::string>(fRef+bandwidth));
      return result;
    }
    const double fReference_;          // nominal frequency / Hz
    const double bandwidth_;           // bandwidth / Hz
  } ;
} // namespace Action

#endif // _FFT_ACTION_AVERAGE_DENSITY_HPP_cm101026_
