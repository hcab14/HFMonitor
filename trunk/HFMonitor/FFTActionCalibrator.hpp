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

#include "Spectrum.hpp"
#include "FFTResultCalibration.hpp"
#include "FFTProxy.hpp"
#include "FFTAction.hpp"

namespace Action {
  class Calibrator : public Base {
  public:
    Calibrator(const boost::property_tree::ptree& config)
      : Base("Calibrator")
      , resultKey_(config.get<std::string>("Name")) {
      using boost::property_tree::ptree;
      const ptree& pt(config.get_child("Inputs"));
      for (ptree::const_iterator i(pt.begin()); i!=pt.end(); ++i) {
        if (i->first == "Input") {
          std::cout << "Calibrator::Calibrator Input." << i->second.get<std::string>("") << std::endl;
          inputs_.push_back(i->second.get<std::string>(""));
        } else {
          std::cout << "Calibrator::calibrate unknown field " << i->first << std::endl;
        }
      }
    }
    virtual ~Calibrator() {}
    virtual void perform(Proxy::Base& p, const SpectrumBase& s) {
      std::cout << "Calibrator::perform" << std::endl;
      // count data
      std::vector<Result::SpectrumPeak::Handle> peaks;
      for (std::vector<std::string>::const_iterator i(inputs_.begin()); i!=inputs_.end(); ++i) {
        try {
          Result::SpectrumPeak::Handle sp(boost::dynamic_pointer_cast<Result::SpectrumPeak>(p.getResult(*i)));
          if (sp != 0) {
            peaks.push_back(sp);
          }
        } catch (...) {
          // TODO ...
        }
      }
      try {     
        Result::Base::Handle rh(new Result::Calibration(peaks));
        p.putResult(resultKey_, rh); 
      } catch (...) {
        // ...
      }
    }
  private:
    std::string resultKey_;           // result key name
    std::vector<std::string> inputs_; // key names of inputs used for calibration
  } ;
} // namespace Action

#endif // _FFT_ACTION_CALIBRATOR_HPP_cm101026_
