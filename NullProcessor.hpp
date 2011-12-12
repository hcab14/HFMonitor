// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _NULL_PROCESSOR_HPP_cm100729_
#define _NULL_PROCESSOR_HPP_cm100729_

#include <iostream>
#include <vector>
#include <complex>
#include <boost/property_tree/ptree.hpp>
#include <boost/integer.hpp>

#include "logging.hpp"
#include "protocol.hpp"

class NullProcessor {
public:
  NullProcessor(const boost::property_tree::ptree& config) {}
  ~NullProcessor() {}

  void procRaw(const Header& header,
               std::vector<char>::const_iterator i0,
               std::vector<char>::const_iterator i1) {
    LOG_INFO(str(boost::format("NullProcessor::procRaw %s") % header));
  }
  void procIQ(const Header& header,
              std::vector<std::complex<double> >::const_iterator i0,
              std::vector<std::complex<double> >::const_iterator i1) {
    LOG_INFO(str(boost::format("NullProcessor::procIQ %s") % header));
  }
protected:
private:
} ;

#endif // _NULL_PROCESSOR_HPP_cm100729_

