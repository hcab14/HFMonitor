// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "protocol.hpp"

#include "ClientFileWAVE.hpp"
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
    config.put("FFTProcessor.Input.<xmlattr>.fileNamePattern", std::string(argv[1]));

    ClientFileWAVE<FFTProcessor<float> > c(config.get_child("FFTProcessor"));
    while (c.process()) 
      ;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }  
  return 0;
}
