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
