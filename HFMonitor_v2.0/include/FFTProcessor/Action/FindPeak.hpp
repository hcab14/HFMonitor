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
#include "FFTProcessor/Result/SpectrumPeak.hpp"
#include "FFTProcessor/Result/Calibration.hpp"
#include "FFTProcessor/Result/CalibratedSpectrumPeak.hpp"
#include "FFTProcessor/Result/PowerSpectrumLine.hpp"
#include "FFTProcessor/Proxy.hpp"
#include "FFTProcessor/Action/SpectrumInterval.hpp"

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
