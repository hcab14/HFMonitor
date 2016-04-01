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
#ifndef _FFT_ACTION_FACTORY_HPP_cm150217_
#define _FFT_ACTION_FACTORY_HPP_cm150217_

#include "FFTProcessor/Action.hpp"
#include "FFTProcessor/Action/Calibrator.hpp"
#include "FFTProcessor/Action/FindPeak.hpp"
#include "FFTProcessor/Action/AverageDensity.hpp"
#include "FFTProcessor/Action/PhaseDifferentiation.hpp"
#include "FFTProcessor/Action/FSKStrength.hpp"

#include "logging.hpp"

namespace Action {
  struct Factory {
    template<typename T>
    static typename Base<T>::Handle makeAction(std::string name,
                                               const boost::property_tree::ptree& pt) {
      typedef typename Base<T>::Handle handle_type;
      if (name == "FindPeak")
        return handle_type(new FindPeak<T>(pt));
      if (name == "Calibrator")
        return handle_type(new Calibrator<T>(pt));
      if (name == "AverageDensity")
        return handle_type(new AverageDensity<T>(pt));
      if (name == "PhaseDifferentiation")
        return handle_type(new PhaseDifferentiation<T>(pt));
      if (name == "FSKStrength")
        return handle_type(new FSKStrength<T>(pt));
      else
        throw std::runtime_error(THROW_SITE_INFO(name+": action not supported"));
    }
  } ;
} // namespace Action 

#endif // _FFT_ACTION_FACTORY_HPP_cm150217_
