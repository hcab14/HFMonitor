// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include "FFTProcessor.hpp"
#include "logging.hpp"
#include "processor.hpp"

namespace processor {
  processor::sptr make_processor(std::string name,
				 const boost::property_tree::ptree& config) {
    processor::sptr result;
    if (name == "FFTProcessor_FLOAT")
      result = FFTProcessor<float>::make(config);
    else if (name == "FFTProcessor_DOUBLE")
      result = FFTProcessor<double>::make(config);
    else
      throw std::runtime_error(THROW_SITE_INFO(name+": processor not implemented"));
    return result;
  }
} // namespace processor
