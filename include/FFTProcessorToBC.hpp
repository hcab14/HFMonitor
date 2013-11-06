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
#ifndef _FFT_PROCESSOR_TO_BC_HPP_cm130210_
#define _FFT_PROCESSOR_TO_BC_HPP_cm130210_

#include "FFTProcessor.hpp"
#include "network/broadcaster.hpp"

template<typename FFTFloat>
class FFTProcessorToBC : public FFTProcessor<FFTFloat> {
public:
  FFTProcessorToBC(const boost::property_tree::ptree& config)
    : FFTProcessor<FFTFloat>(config)
    , broadcaster_(broadcaster::make(config.get_child("Broadcaster")))
    , started_(false)
    , station_info_(config.get<std::string>("StationInfo")) {}

  virtual ~FFTProcessorToBC() {}

protected:
  typedef typename FFTProcessor<FFTFloat>::ResultMap ResultMap;
  virtual void dump(const typename ResultMap::value_type& result) {
    if (not started_) {
      broadcaster_->start();
      started_= true;
    }
    const std::string path(result.second->name());
    result.second->dumpToBC(path, result.first, broadcaster_, station_info_);
  }
private:
  broadcaster::sptr broadcaster_;
  bool              started_;
  std::string       station_info_;
} ;

#endif // _FFT_PROCESSOR_TO_BC_HPP_cm130210_

