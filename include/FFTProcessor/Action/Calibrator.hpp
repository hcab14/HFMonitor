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
#ifndef _FFT_ACTION_CALIBRATOR_HPP_cm101026_
#define _FFT_ACTION_CALIBRATOR_HPP_cm101026_

#include <iostream>
#include <string>
#include <stdexcept>
#include <boost/noncopyable.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>

#include "Spectrum.hpp"
#include "FFTProcessor/Result/Calibration.hpp"
#include "FFTProcessor/Proxy.hpp"
#include "FFTProcessor/Action.hpp"

namespace Action {
  class Calibrator : public Base {
  public:
    Calibrator(const boost::property_tree::ptree& config)
      : Base("Calibrator")
      , resultKey_(config.get<std::string>("<xmlattr>.name"))
      , offsetMax_(config.get<double>("<xmlattr>.maxOffset_Hz")) 
      , ppmMax_   (config.get<double>("<xmlattr>.maxCorrectionFactor_ppm")) {
      using boost::property_tree::ptree;
      BOOST_FOREACH(const ptree::value_type input, config.get_child("Inputs")) {
        if (input.first == "Input") {
          LOG_INFO(str(boost::format("Calibrator::Calibrator Input.%s") 
                       % input.second.get<std::string>("<xmlattr>.key")));
          inputs_.push_back(input.second.get<std::string>("<xmlattr>.key"));
        } else {
          LOG_WARNING(str(boost::format("Calibrator::calibrate unknown field %s") % input.first));
        }
      }
    }
    virtual ~Calibrator() {}
    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      // count data
      std::vector<Result::SpectrumPeak::Handle> peaks;
      BOOST_FOREACH(const std::string& input, inputs_) {
        try {
          Result::SpectrumPeak::Handle 
            sp(boost::dynamic_pointer_cast<Result::SpectrumPeak>(p.getResult(input)));
          if (sp != 0)
            peaks.push_back(sp);
        } catch (const std::exception& e) {
          LOG_WARNING(e.what());
        }
      }
      try {     
        Result::Base::Handle rh(new Result::Calibration(p.getApproxPTime(), peaks,
                                                        offsetMax_, ppmMax_));
        p.putResult(resultKey_, rh); 
      } catch (const std::exception& e) {
        LOG_WARNING(e.what());
      }
    }
  private:
    std::string resultKey_;           // result key name
    double offsetMax_;                // max offset [Hz]
    double ppmMax_;                   // max ppm
    std::vector<std::string> inputs_; // key names of inputs used for calibration
  } ;
} // namespace Action

#endif // _FFT_ACTION_CALIBRATOR_HPP_cm101026_
