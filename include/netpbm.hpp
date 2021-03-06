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
#ifndef _NET_PBM_HPP_cm101026_
#define _NET_PBM_HPP_cm101026_

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "logging.hpp"

namespace netpbm {
  class pgm_writer : private boost::noncopyable {
  public:
    typedef std::vector<std::string> string_vector;
    typedef boost::shared_ptr<pgm_writer> sptr;

    pgm_writer(size_t width,
               boost::filesystem::fstream& os) 
      : os_(os)
      , width_(width)
      , height_(0)
      , need_height_update_(false) {}
    
    boost::filesystem::fstream& write_header() {
      write_str("P5\n");
      write_str(str(boost::format("%9d")  % width_));
      height_pos_= os_.tellp();
      write_str(str(boost::format(" %9d\n")  % height_));
      write_str("255\n");
      return os_;
    }

    boost::filesystem::fstream& read_header() {
      os_.seekp(0, std::ios::beg);
      std::string pgm_magic; os_ >> pgm_magic;
      ASSERT_THROW(pgm_magic == "P5");
      size_t width; os_ >> width;
      ASSERT_THROW(width == width_);
      height_pos_= os_.tellg();
      os_ >> height_;
      os_.seekp(0, std::ios::end);
      return os_;
    }    

    ~pgm_writer() {
      update_height();
    }
    
    size_t width() const { return width_; }
    size_t height() const { return height_; }
    
    bool write_line(std::string s, bool perform_height_update=false) {
      if (!os_ || s.size() != width_) return false;
      if (not write_str(s, false)) return false;
      ++height_;
      need_height_update_= true;
      if (perform_height_update)
	update_height();
      return true;
    }
    
  private:
    void update_height() {
      if (not need_height_update_) return;
      os_.seekp(height_pos_, std::ios::beg);
      write_str(str(boost::format(" %9d\n")  % height_));
      os_.seekp(0, std::ios::end);
      need_height_update_= false;
    }
    
    bool write_str(std::string s, bool throw_on_error=true) { 
      bool success(false);
      try {
	success= bool(os_.write(s.c_str(), s.size()));
      } catch (const std::runtime_error& error) {
	if (throw_on_error) throw error; else return false;
      }
      if (throw_on_error && not success) 
	throw std::runtime_error(THROW_SITE_INFO("write_str failed"));
      return success;
    }

    boost::filesystem::fstream& os_;
    const size_t width_;
    size_t height_;
    std::ostream::streampos height_pos_;
    bool need_height_update_;
  } ;
    
} // namespace netpbm

#endif // _NET_PBM_HPP_cm101026_

