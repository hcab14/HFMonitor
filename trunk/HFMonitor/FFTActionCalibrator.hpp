// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
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
#include "FFTResultCalibration.hpp"
#include "FFTProxy.hpp"
#include "FFTAction.hpp"

namespace Action {
  class Calibrator : public Base {
  public:
    Calibrator(const boost::property_tree::ptree& config)
      : Base("Calibrator")
      , resultKey_(config.get<std::string>("<xmlattr>.name")) {
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
          if (sp != 0) {
            peaks.push_back(sp);
          }
        } catch (const std::runtime_error& e) {
          LOG_WARNING(e.what());
        }
      }
      try {     
        Result::Base::Handle rh(new Result::Calibration(p.getApproxPTime(), peaks));
        p.putResult(resultKey_, rh); 
      } catch (const std::runtime_error& e) {
        LOG_WARNING(e.what());
      }
    }
  private:
    std::string resultKey_;           // result key name
    std::vector<std::string> inputs_; // key names of inputs used for calibration
  } ;
} // namespace Action

#endif // _FFT_ACTION_CALIBRATOR_HPP_cm101026_
