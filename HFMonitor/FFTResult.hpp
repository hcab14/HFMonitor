// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_HPP_cm101026_
#define _FFT_RESULT_HPP_cm101026_

#include <string>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "gen_filename.hpp"

namespace Result {
  class Base : public gen_filename {
  public:
    typedef boost::shared_ptr<Base> Handle;

    Base(std::string name) 
      : name_(name)
      , posEndTime_(0) {}
    virtual ~Base() {}
    virtual std::string toString() const { return name(); }
    std::string name() const { return name_; }

    friend std::ostream& operator<<(std::ostream& os, const Base& b) {
      return os << b.toString();
    }

    void dump(std::string path,
              std::string tag,
              boost::posix_time::ptime t) const {      
      boost::filesystem::path p(gen_file_path(path, tag, t));
      const bool file_exists(boost::filesystem::exists(p));
      if (not file_exists) {
        boost::filesystem::fstream ofs(p, std::ios::out);
        dumpHeader(ofs, t) << lineBreak();
      } else {
        findPositionOfEndTime(p);
      }
      boost::filesystem::fstream ofs(p, std::ios::in | std::ios::out);
      ofs.seekp(0, std::ios::end);
      dumpData(ofs, t) << lineBreak();
    }
  protected:
    virtual std::string lineBreak() const { return "\n"; }
    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os,
                                                   boost::posix_time::ptime t) const {
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%s")));
      oss << t;
      os << "# StartTime = " << oss.str() << " [UTC]" << lineBreak();
      os << "# EndTime   = " << oss.str() << " [UTC]" << lineBreak();
      os << "# Time_UTC ";
      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os,
                                                 boost::posix_time::ptime t) const {
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%s")));
      oss << t;
      // updateTimeTag(os, posEndTime_, oss.str());
      os << oss.str() << " ";
      return os;
    }
  protected:
    std::string name_;

  private:
    void findPositionOfEndTime(const boost::filesystem::path& p) const {
      boost::filesystem::ifstream ifs(p);
      std::string line;
      std::ostream::streampos pos(0);
      while (std::getline(ifs, line)) {
        if (line[0] != '#') break;
        pos += line.size();
        if (std::string(line, 2, 9) == "EndTime  ")
          posEndTime_   = pos + std::streamoff(14);
      }
    }
    void updateTimeTag(boost::filesystem::fstream& os,
                    std::ostream::streampos pos,
                    std::string timeTag) const {
      os.seekp(pos, std::ios::beg);
      os << timeTag;
      os.seekp(0,   std::ios::end);
    }

    mutable std::ostream::streampos posEndTime_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_HPP_cm101026_
