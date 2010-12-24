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
      , firstEpoch_(true)
      , posStartTime_(0)
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
        firstEpoch_ = true;
      } else {
        findPositionOfStartEndTime(p);
        firstEpoch_ = false;
      }
      boost::filesystem::fstream ofs(p, std::ios::in | std::ios::out);
      ofs.seekp(0, std::ios::end);
      dumpData(ofs, t) << lineBreak();
    }
  protected:
    virtual std::string lineBreak() const { return "\n"; }
    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os,
                                                   boost::posix_time::ptime t) const {
      os << "# StartTime = ";
      posStartTime_ = os.tellp();
      os << "YYYY-MM-DD HH:MM:SS.ssssss [UTC]\n";

      os << "# EndTime   = ";
      posEndTime_ = os.tellp();
      os << "YYYY-MM-DD HH:MM:SS.ssssss [UTC]\n";
      os << "# Time_UTC ";

      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os,
                                                 boost::posix_time::ptime t) const {
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%s")));
      oss << t;
      if (firstEpoch_)
        updateTimeTag(os, posStartTime_, oss.str()); 
      updateTimeTag(os, posEndTime_, oss.str());
      firstEpoch_ = false;
      os << oss.str() << " ";
      return os;
    }
  protected:
    std::string name_;

  private:
    void findPositionOfStartEndTime(const boost::filesystem::path& p) const {
      boost::filesystem::ifstream ifs(p);
      std::string line;
      std::istream::streampos pos(0);
      while (std::getline(ifs, line)) {
        if (line[0] != '#') break;
        if (std::string(line, 2, 9) == "StartTime")
          posStartTime_ = pos + std::streamoff(14);
        if (std::string(line, 2, 9) == "EndTime  ")
          posEndTime_   = pos + std::streamoff(14);
        pos = ifs.tellg();
      }
    }
    void updateTimeTag(boost::filesystem::fstream& os,
                    std::ostream::streampos pos,
                    std::string timeTag) const {
      os.seekp(pos, std::ios::beg);
      os << timeTag;
      os.seekp(0,   std::ios::end);
    }

    mutable bool firstEpoch_;
    mutable std::ostream::streampos posStartTime_;
    mutable std::ostream::streampos posEndTime_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_HPP_cm101026_
