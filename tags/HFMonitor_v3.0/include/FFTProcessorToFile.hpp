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
#ifndef _FFT_PROCESSOR_TO_FILE_HPP_cm130310_
#define _FFT_PROCESSOR_TO_FILE_HPP_cm130310_

#include <boost/asio.hpp>

#include "FFTProcessor.hpp"

template<typename FFTFloat>
class FFTProcessorToFile : public FFTProcessor<FFTFloat> {
public:
  FFTProcessorToFile(const boost::property_tree::ptree& config)
    : FFTProcessor<FFTFloat>(config)
    , dataPath_(config.get<std::string>("FileSink.<xmlattr>.path"))
    , station_info_(config.get<std::string>("StationInfo")) {}

  virtual ~FFTProcessorToFile() {}

  boost::asio::io_service& get_service() { return service_; }
  
protected:
  typedef typename FFTProcessor<FFTFloat>::ResultMap ResultMap;

  virtual void dump(const typename ResultMap::value_type& result) {
    result.second->dumpToFile(dataPath_, result.first, station_info_);
  }
private:
  std::string             dataPath_;
  boost::asio::io_service service_;
  std::string             station_info_;
} ;


#endif // _FFT_PROCESSOR_TO_FILE_HPP_cm130310_

