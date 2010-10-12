// -*- C++ -*-
// $Id$
#ifndef _NULL_PROCESSOR_HPP_cm100729_
#define _NULL_PROCESSOR_HPP_cm100729_

#include <iostream>
#include <vector>
#include <complex>
#include <boost/property_tree/ptree.hpp>
#include <boost/integer.hpp>
#include "protocol.hpp"

class NullProcessor {
public:
  NullProcessor(const boost::property_tree::ptree& config) {}
  ~NullProcessor() {}

  void procRaw(const Header& header, const std::vector<char>& rawData) { 
    std::cout << "NullProcessor::procRaw " << header << std::endl;
  }
  void procIQ(const Header& header, 
	std::vector<std::complex<double> >::const_iterator i0,
        std::vector<std::complex<double> >::const_iterator i1 ) { 
    std::cout << "NullProcessor::procIQ " << header << std::endl;
  }
protected:
private:
} ;

#endif // _NULL_PROCESSOR_HPP_cm100729_

