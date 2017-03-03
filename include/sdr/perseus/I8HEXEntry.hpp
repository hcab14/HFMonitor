// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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
#ifndef _I8HEXEntry_hpp_cm120321_
#define _I8HEXEntry_hpp_cm120321_

#include <vector>
#include <iterator>
#include <numeric>
#include <string>
#include <boost/format.hpp>
#include "logging.hpp"

namespace Perseus {
  class I8HEXEntry {
  public:
    typedef enum Type {
      DataRecord      = 0,
      EndOfFileRecord = 1
    } Type;
    typedef std::vector<boost::uint8_t> code_vector;

    I8HEXEntry(std::string line) {
      ASSERT_THROW(line[0] == ':');
      ASSERT_THROW(line.size() > 9);
      _addr = hex2int(std::string(line,3,4));
      const boost::uint8_t type(hex2int(std::string(line,7,2)));
      ASSERT_THROW(type == DataRecord || type == EndOfFileRecord);
      if (type == DataRecord)      _type = DataRecord;
      if (type == EndOfFileRecord) _type = EndOfFileRecord;
      const size_t length(hex2int(std::string(line,1,2)));
      for (size_t i=0; i<length; ++i)
        _code.push_back(hex2int(std::string(line,9+2*i,2)));
      ASSERT_THROW(hex2int(std::string(line,9+2*length,2)) == checksum());
    }
    std::string to_str() const {
      std::string line(str(boost::format(":%02X%04X%02X") 
                           % _code.size()
                           % int(_addr)
                           % _type));
      for (boost::uint8_t c : _code)
        line += str(boost::format("%02X") % int(c));
      return line + str(boost::format("%02X") % int(checksum()));
    }

    Type get_type() const { return _type; }
    boost::uint16_t get_addr() const { return _addr; }
    const code_vector& get_code() const { return _code; }
  protected:
  private:
    boost::uint8_t checksum() const { // = 2-compement of sum of all bytes
      return 1 + (0xFF ^ (_code.size()
                          + _type
                          + (_addr&0xFF)
                          + (_addr>>8)
                          + std::accumulate(_code.begin(), _code.end(), 0)));
    }
    static unsigned int hex2int(const std::string s) {
      unsigned result(0);
      std::stringstream iss; iss << std::hex << s; iss >> result;
      return result;
    }
    boost::uint16_t _addr;
    Type            _type;
    code_vector     _code;
  } ;
} // namespace Perseus
#endif // _I8HEXEntry_hpp_cm120321_
