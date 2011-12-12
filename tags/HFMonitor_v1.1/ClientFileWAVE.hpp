// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _CLIENT_FILE_WAVE_HPP_cm100925_
#define _CLIENT_FILE_WAVE_HPP_cm100925_

#include <iostream>
#include <vector>
#include <complex>

#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/property_tree/ptree.hpp>

#include "logging.hpp"
#include "WAVE.hpp"

template<typename PROCESSOR>
class ClientFileWAVE : private boost::noncopyable {
public:
  ClientFileWAVE(const boost::property_tree::ptree& config)
    : p_(config)
    , fileNamePattern_(config.get<std::string>("Input.<xmlattr>.fileNamePattern"))
    , fileNumber_(0)
    , bufferLengthSec_(config.get<double>("Repack.<xmlattr>.bufferLength_sec"))
    , overlap_(0.01*config.get<double>("Repack.<xmlattr>.overlap_percent"))
    , sampleNumber_(0) {
  }

  bool process() {
    boost::filesystem::path p(boost::str(boost::format(fileNamePattern_) % fileNumber_));    
    LOG_INFO(str(boost::format("ClientFileWAVE::process() p = %s") % p));
    if (!boost::filesystem::exists(p)) 
      return false;

    WAVE::ProcessFile<PROCESSOR> pf(p_, p.string(), sampleNumber_, samples_, bufferLengthSec_, overlap_);
    while (pf.proc() > 0) ;
    sampleNumber_ = pf.sampleNumber();
    samples_      = pf.samples();
    fileNumber_++;
    return true;
  }

protected:
private:
  PROCESSOR       p_;
  std::string     fileNamePattern_;
  boost::uint32_t fileNumber_;
  double          bufferLengthSec_;  
  double          overlap_;  
  boost::uint64_t sampleNumber_;
  std::vector<std::complex<double> > samples_;
} ;

#endif // _CLIENT_FILE_WAVE_HPP_cm100925_
