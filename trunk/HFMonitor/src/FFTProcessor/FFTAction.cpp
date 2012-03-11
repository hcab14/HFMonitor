// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include "FFTProcessor/Action.hpp"
#include "FFTProcessor/Action/Calibrator.hpp"
#include "FFTProcessor/Action/FindPeak.hpp"
#include "FFTProcessor/Action/AverageDensity.hpp"
#include "FFTProcessor/Action/PhaseDifferentiation.hpp"
#include "FFTProcessor/Action/FSKStrength.hpp"

#include "logging.hpp"

namespace Action {
  Handle Factory::makeAction(std::string name,
                             const boost::property_tree::ptree& pt) {
    if (name == "FindPeak")
      return Handle(new FindPeak(pt));
    if (name == "Calibrator")
      return Handle(new Calibrator(pt));
    if (name == "AverageDensity")
      return Handle(new AverageDensity(pt));
    if (name == "PhaseDifferentiation")
      return Handle(new PhaseDifferentiation(pt));
    if (name == "FSKStrength")
      return Handle(new FSKStrength(pt));
    else
      throw std::runtime_error(THROW_SITE_INFO(name+": action not supported"));
  }
} // namespace Action 
