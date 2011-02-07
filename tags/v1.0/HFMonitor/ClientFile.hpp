// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _CLIENT_FILE_HPP_cm100816_
#define _CLIENT_FILE_HPP_cm100816_

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "logging.hpp"
#include "protocol.hpp"

template<typename PROCESSOR>
class ClientFile : private boost::noncopyable {
public:
  ClientFile(const boost::property_tree::ptree& config)
    : p_(config)
    , fileNamePattern_(config.get<std::string>("Input.<xmlattr>.fileNamePattern"))
    , fileNumber_(0) {
  }

  bool process() {
    boost::filesystem::path p(boost::str(boost::format(fileNamePattern_) % fileNumber_));    
    LOG_INFO(str(boost::format("ClientFile::process() p = %s") % p));
    if (!boost::filesystem::exists(p)) 
      return false;
    else {
      for (boost::filesystem::ifstream ifs(p, std::ios::binary | std::ios::in); ifs;) {
        Header header;
        if (!ifs.read((char*)&header, sizeof(Header))) 
          break;
        std::vector<char> data(6*header.numberOfSamples());
        if (!ifs.read(data.begin().operator->(), data.size())) 
          return false;
        p_.procRaw(header, data.begin(), data.end());
      }
      fileNumber_++;
    }
    return true;
  }

protected:  
private:
  PROCESSOR       p_;
  std::string     fileNamePattern_;
  boost::uint32_t fileNumber_;
} ;

#endif // _CLIENT_FILE_HPP_cm100729_
