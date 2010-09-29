// -*- C++ -*-
// $Id$
#ifndef _DUMP_PROCESSOR_HPP_cm100729_
#define _DUMP_PROCESSOR_HPP_cm100729_

#include <iostream>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/integer.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "protocol.hpp"

class DumpProcessor {
public:
  DumpProcessor(const boost::property_tree::ptree& config)
    : sampleCounter_(0)
    , samplesPerFile_(config.get<unsigned>("Dump.SamplesPerFile"))
    , fileNamePattern_(config.get<std::string>("Dump.FileNamePattern"))
    , fileNumber_(0) {}
  ~DumpProcessor() {}

  void procRaw(const Header& header, const std::vector<char>& data) { 
    std::cout << "procRaw " << header << std::endl;
    if (sampleCounter_ > samplesPerFile_) {
      sampleCounter_= 0;
      ++fileNumber_;
    }
    boost::filesystem::path p(boost::str(boost::format(fileNamePattern_) % fileNumber_));
    if (sampleCounter_ == 0 && boost::filesystem::exists(p))
      boost::filesystem::remove(p);
    boost::filesystem::ofstream ofs(p, std::ios::binary | std::ios::app);
    ofs.write((const char*)&header, sizeof(Header));
    ofs.write(&data[0], data.size());
    sampleCounter_ += header.numberOfSamples();
  }

protected:
private:
  boost::uint32_t sampleCounter_;
  boost::uint32_t samplesPerFile_;
  std::string     fileNamePattern_;
  boost::uint32_t fileNumber_;
} ;

#endif

