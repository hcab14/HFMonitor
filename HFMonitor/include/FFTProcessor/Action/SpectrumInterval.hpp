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
#ifndef _FFT_ACTION_SPECTRUM_INTERVAL_HPP_cm101026_
#define _FFT_ACTION_SPECTRUM_INTERVAL_HPP_cm101026_

#include <iostream>
#include <string>
#include <stdexcept>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "Spectrum.hpp"
#include "FFTProcessor/Filter.hpp"
#include "FFTProcessor/Result.hpp"
#include "FFTProcessor/Proxy.hpp"

#include "logging.hpp"

namespace Action {
  template<typename T>
  class SpectrumInterval : public Base<T> {
  public:
    typedef frequency_vector<double> PowerSpectrum;
    typedef Filter::Cascaded<PowerSpectrum> filter_type;
    SpectrumInterval(const boost::property_tree::ptree& config)
      : Base<T>("SpectrumInterval")
      , fMin_(config.get<double>("<xmlattr>.fMin_Hz"))
      , fMax_(config.get<double>("<xmlattr>.fMax_Hz"))
      , resultKey_(config.get<std::string>("<xmlattr>.name"))
      , useCalibration_(config.find("Calibration") != config.not_found())
      , calibrationKey_(useCalibration_ ? config.get<std::string>("Calibration.<xmlattr>.key") : "")
      , plotSpectrum_(config.get<bool>("<xmlattr>.plotSpectrum", false)) {
      if (config.find("Filter") != config.not_found()) {
        if (config.get<std::string>("Filter.<xmlattr>.type") == "None") {
          // nop
        } else if (config.get<std::string>("Filter.<xmlattr>.type") == "LowPass") {
          filter_.add(Filter::LowPass<PowerSpectrum>::make
                      (1.0, config.get<double>("Filter.<xmlattr>.timeConstant_sec")));
        } else {
          throw std::runtime_error
            (THROW_SITE_INFO(config.get<std::string>("Filter.<xmlattr>.type") + ": unknown filter"));
        }
      }
    }

    virtual ~SpectrumInterval() {}    

    std::string resultKey() const { return resultKey_; }
    bool useCalibration() const { return useCalibration_; }
    std::string calibrationKey() const { return calibrationKey_; }
    bool plotSpectrum() const { return plotSpectrum_; }

    virtual void perform(Proxy::Base& p, const T& s) {
      try {
        const PowerSpectrum ps(fMin_, fMax_, s, std::abs<double>);        
        if (filter_.x().empty()) {
          filter_.init(p.getApproxPTime(), ps);
        } else {
          filter_.update(p.getApproxPTime(), ps);
        }
        // call virtual method
        proc(p, s, filter_.x());
      } catch (const std::exception& e) {
        LOG_WARNING(e.what());
      } catch (...) {
        LOG_WARNING("SpectrumInterval::perform unknown error");
      }
    }

    // this method is overwritten by, e.g., FindPeak, see below
    virtual void proc(Proxy::Base& p, 
                      const T& s,
                      const PowerSpectrum& ps) = 0;
    
  protected:
  private:
    filter_type filter_;
    const double fMin_;
    const double fMax_;
    const std::string resultKey_;
    const bool useCalibration_;
    const std::string calibrationKey_;
    const bool plotSpectrum_;
  } ;
} // namespace Action

#endif // _FFT_ACTION_SPECTRUM_INTERVAL_HPP_cm101026_
