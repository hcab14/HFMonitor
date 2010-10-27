// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
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

namespace netpbm {
  class pgm_writer : private boost::noncopyable {
  public:
    typedef std::vector<std::string> string_vector;
    typedef boost::shared_ptr<pgm_writer> sptr;

    pgm_writer(size_t width,
               boost::filesystem::fstream& os) 
      : file_exists_(true)
      , os_(os)
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
      if (pgm_magic != "P5") 
        throw std::runtime_error("PGMWriter::PGMWriter tying to append to a non-PGM file :" + pgm_magic);
      size_t width; os_ >> width;
      if (width != width_)
        throw std::runtime_error("PGMWriter::PGMWriter width != width_");
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
	success= os_.write(s.c_str(), s.size());
      } catch (const std::runtime_error& error) {
	if (throw_on_error) throw error; else return false;
      }
      if (throw_on_error && not success) 
	throw std::runtime_error("write_str failed");
      return success;
    }

    const bool file_exists_;
    boost::filesystem::fstream& os_;
    const size_t width_;
    size_t height_;
    std::ostream::streampos height_pos_;
    bool need_height_update_;
  } ;
    
} // namespace netpbm

#endif // _NET_PBM_HPP_cm101026_

