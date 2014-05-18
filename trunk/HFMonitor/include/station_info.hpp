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
#ifndef _STATION_INFO_HPP_cm130704_
#define _STATION_INFO_HPP_cm130704_

#include <sstream>
#include <string>

class station_info {
public:
  station_info(std::string s,
               std::string line_break="\n")
    : s_(format(s, line_break)) {}

  std::string to_string() const { return s_; }

  friend std::ostream& operator<<(std::ostream& os, const station_info& si) {
    return os << si.to_string();
  }
  
protected:
  static std::string format(std::string s, std::string line_break) {
    std::stringstream si(s); // input stream
    std::stringstream r;    // result stream
    std::string line;
    while (getline(si, line)) {
      // skip white space
      while (!line.empty() && line[0] == ' ')
        line = std::string(line, 1);
      if (!line.empty())
        r << "# " << line << line_break;
    }
    return r.str();
  }

private:
  std::string s_;
} ;

#endif // _STATION_INFO_HPP_cm130704_
