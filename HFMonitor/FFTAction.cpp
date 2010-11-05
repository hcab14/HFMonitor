// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include "FFTAction.hpp"
#include "FFTActionFindPeak.hpp"
#include "FFTActionCalibrator.hpp"
#include "FFTActionAverageDensity.hpp"

namespace Action {
  Handle Factory::makeAction(std::string name,
                             const boost::property_tree::ptree& pt) {
    if (name == "FindPeak")
      return Handle(new FindPeak(pt));
    if (name == "Calibrator")
      return Handle(new Calibrator(pt));
    if (name == "AverageDensity")
      return Handle(new AverageDensity(pt));
    else
      throw std::runtime_error(name+": action not supported");
  }
} // namespace Action 
