// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "protocol.hpp"

#include "ClientFile.hpp"
#include "Raw2IQAdapter.hpp"
#include "RepackProcessor.hpp"
#include "FFTProcessor.hpp"
#include "NullProcessor.hpp"

int main(int argc, char* argv[])
{
  try {
    if (argc < 2) {
      std::cerr << "### Usage: client <FileNamePattern> <config.xml>" << std::endl;
      return 1;
    }   
    boost::property_tree::ptree config;
    std::string filename((argc > 2 ) ? argv[2] : "config.xml");
    read_xml(filename, config);    
    config.put("FileNamePattern", std::string(argv[1]));

    ClientFile<Raw2IQAdapter<RepackProcessor<FFTProcessor<float> > > > c(config);
    while (c.process()) 
      ;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }  
  return 0;
}
