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
#include <boost/weak_ptr.hpp>

#include "gen_filename.hpp"

namespace Result {
  class Base : public gen_filename {
  public:
    typedef boost::shared_ptr<Base> Handle;
    typedef boost::weak_ptr<Base> WeakHandle;
    typedef boost::posix_time::ptime ptime;
    typedef std::vector<Handle> HandleVector;

    Base(std::string name, ptime time)
      : name_(name)
      , time_(time)
      , posEndTime_(0) {}
    virtual ~Base() {}
    virtual std::string toString() const { return name(); }
    std::string name() const { return name_; }
    ptime time() const { return time_; }

    friend std::ostream& operator<<(std::ostream& os, const Base& b) {
      return os << b.toString();
    }

    void push_back(const Handle& h) { handles_.push_back(h); }
    
    const Base* getLatest() const { 
      return handles_.empty() ? this : handles_.back().get();
    }

    // dump data
    void dump(std::string path,
              std::string tag) const {      
      boost::filesystem::fstream *ofs(0);
      ofs = dumpSingle(ofs, path, tag, this);
      BOOST_FOREACH(const HandleVector::value_type& h, handles_)
        ofs = dumpSingle(ofs, path, tag, h.get());

      updateTimeTag(*ofs, posEndTime_, getLatest()->makeTimeLabel());
      handles_.clear();
      delete ofs;
    }
    boost::filesystem::fstream* dumpSingle(boost::filesystem::fstream* ofs, 
                                           std::string path, std::string tag, 
                                           const Base* h) const {
      const boost::filesystem::path p(gen_file_path(path, tag, h->time()));
      const bool file_exists(boost::filesystem::exists(p));
      if (not file_exists) {
        if (ofs != 0) { 
          delete ofs; ofs=0; 
        }
        boost::filesystem::fstream lofs(p, std::ios::out);  
        h->dumpHeader(lofs) << h->lineBreak();
      }
      if (ofs == 0) {
        findPositionOfEndTime(p);
        ofs = new boost::filesystem::fstream(p, std::ios::in | std::ios::out);
        ofs->seekp(0, std::ios::end);
      }
      h->dumpData(*ofs) << h->lineBreak();
      return ofs;
    }
  protected:
    virtual std::string lineBreak() const { return "\n"; }
    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os) const {
      std::string timeLabel(makeTimeLabel());
      os << "# StartTime = " << timeLabel << " [UTC]" << lineBreak();
      posEndTime_   = os.tellg() + std::streamoff(14);          
      os << "# EndTime   = " << timeLabel << " [UTC]" << lineBreak();
      os << "# Time_UTC ";
      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os) const {
      os << makeTimeLabel() << " ";
      return os;
    }
    virtual updateTimeTag(boost::filesystem::fstream& os,
                          std::ostream::streampos pos,
                          std::string timeTag) const {
      os.seekp(pos, std::ios::beg);
      os << timeTag;
      os.seekp(0,   std::ios::end);
    }
  protected:
    std::string name_;
    ptime time_;
  private:
    std::string makeTimeLabel() const {
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%s")));
      oss << time();
      return oss.str();
    }
    void findPositionOfEndTime(const boost::filesystem::path& p) const {
      boost::filesystem::ifstream ifs(p);
      std::string line;
      std::ostream::streampos pos(0);
      while (std::getline(ifs, line)) {
        if (line[0] != '#') break;
        if (std::string(line, 2, 9) == "EndTime  ")
          posEndTime_   = pos + std::streamoff(14);
        pos += line.size();
      }
    }

    mutable std::ostream::streampos posEndTime_;
    // collected results
    mutable HandleVector handles_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_HPP_cm101026_
