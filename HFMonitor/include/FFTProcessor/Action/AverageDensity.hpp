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
#include "FFTProcessor/Result/SpectrumPowerInInterval.hpp"
#include "FFTProcessor/Result/Calibration.hpp"
#include "FFTProcessor/Proxy.hpp"
#include "FFTProcessor/Action/SpectrumInterval.hpp"

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
      try {
        Result::SpectrumPowerInInterval::Handle 
          spp((useCalibration())
              ? boost::make_shared<Result::CalibratedSpectrumPowerInInterval>
              (p.getApproxPTime(), fReference_, bandwidth_, s.normWindow(), 
               boost::dynamic_pointer_cast<Result::Calibration>
               (p.getResult(calibrationKey())))
              : boost::make_shared<Result::SpectrumPowerInInterval>
              (p.getApproxPTime(), fReference_, bandwidth_, s.normWindow()));
        if (spp->proc(p, ps))
          p.putResult(resultKey(), spp);
        if (plotSpectrum())
          p.putResult(resultKey()+"_plot",
                      boost::make_shared<Result::PowerSpectrumLine>(p.getApproxPTime(), ps));
      } catch (const std::exception& e) {
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
