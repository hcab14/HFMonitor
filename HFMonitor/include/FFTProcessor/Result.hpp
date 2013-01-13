// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_HPP_cm101026_
#define _FFT_RESULT_HPP_cm101026_

#include <string>
#include <fstream>
#include <deque>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/weak_ptr.hpp>

#include "network/protocol.hpp"
#include "gen_filename.hpp"

namespace Result {
  class Base : public gen_filename {
  public:
    typedef boost::shared_ptr<Base> Handle;
    typedef boost::weak_ptr<Base> WeakHandle;
    typedef boost::posix_time::ptime ptime;
    typedef std::deque<Handle> HandleVector;

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

    void clear() {
      handles_.clear();
    }

    // dump data
    void dumpToFile(std::string path,
                    std::string tag) {
      boost::filesystem::fstream *ofs(0);
      ofs = dumpSingle(ofs, path, tag, this);
      BOOST_FOREACH(const HandleVector::value_type& h, handles_)
        ofs = dumpSingle(ofs, path, tag, h.get());
      updateTimeTag(*ofs, posEndTime_, getLatest()->makeTimeLabel());
      delete ofs;
    }

    template<typename BROADCASTER>
    void dumpToBC(std::string path,
                  std::string tag,
                  boost::shared_ptr<BROADCASTER> bc) {
      dumpToBCSingle(path, tag, bc, this);
      BOOST_FOREACH(const HandleVector::value_type& h, handles_)
        dumpToBCSingle(path, tag, bc, h.get());
    }

    template<typename BROADCASTER>
    void dumpToBCSingle(std::string path,
                        std::string tag,
                        boost::shared_ptr<BROADCASTER> bc,
                        Base* h) {
      std::ostringstream sData;
      h->dumpData(sData);
//       std::cout << "dumpToBCSingle: " << path << " " << tag << " '" << sData.str() << "'" << std::endl;
//       std::ostringstream sHeader;
//       h->dumpData(sHeader);
      std::string sd(sData.str());
      const boost::uint32_t stream_number(bc->register_stream(tag));
      const header header(h->format(), h->time(), stream_number, sData.str().size());
      std::string d;
      std::copy(header.begin(), header.end(), std::back_inserter(d));
      std::copy(sd.begin(),     sd.end(),     std::back_inserter(d));
//       std::cout << "dumpToBCSingle: " << path << " " << tag << " '" << sData.str()
//                 << "' s=" << d.size() - sizeof(header)
//                 << " sData.size=" << sData.str().size()        
//                 << std::endl;
      bc->bc_data(this->time(), tag, d);//, sHeader.str());
    }

    boost::filesystem::fstream* dumpSingle(boost::filesystem::fstream* ofs, 
                                           std::string path, std::string tag, 
                                           const Base* h) const {
      const boost::filesystem::path p(gen_file_path(path, tag, h->time()));
      const bool file_exists(boost::filesystem::exists(p));
      if (not file_exists) {
        if (ofs != NULL) { 
          delete ofs; ofs= NULL; 
        }
        boost::filesystem::fstream lofs(p, std::ios::out);  
        h->dumpHeaderToFile(lofs) << h->lineBreak();
      }
      if (ofs == NULL) {
        findPositionOfEndTime(p);
        ofs = new boost::filesystem::fstream(p, std::ios::in | std::ios::out);
        ofs->seekp(0, std::ios::end);
      }
      h->dumpDataToFile(*ofs) << h->lineBreak();
      return ofs;
    }
  protected:
    virtual std::string lineBreak() const { return "\n"; }

    // virtual string context dump header and data
    virtual std::ostream& dumpHeader(std::ostream& os) const {
      return os;
    }
    virtual std::ostream& dumpData(std::ostream& os) const {
      return os;
    }

    // file context dump:
    virtual boost::filesystem::fstream& dumpHeaderToFile(boost::filesystem::fstream& os) const {
      std::string timeLabel(makeTimeLabel());
      os << "# StartTime = " << timeLabel << " [UTC]" << lineBreak();
      posEndTime_   = os.tellg() + std::streamoff(14);          
      os << "# EndTime   = " << timeLabel << " [UTC]" << lineBreak();
      os << "# Time_UTC ";
      std::ostringstream oss; dumpHeader(oss);
      os << oss.str();
      return os;
    }
    virtual boost::filesystem::fstream& dumpDataToFile(boost::filesystem::fstream& os) const {
      os << makeTimeLabel() << " ";
      std::ostringstream oss; dumpData(oss);
      os << oss.str();
      return os;
    }

    virtual void updateTimeTag(boost::filesystem::fstream& os,
                               std::ostream::streampos pos,
                               std::string timeTag) const {
      os.seekp(pos, std::ios::beg);
      os << timeTag;
      os.seekp(0,   std::ios::end);
    }

    virtual std::string format() const { return "TXT_0000"; }

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
        pos += line.size() + 1;
      }
    }

    mutable std::ostream::streampos posEndTime_;
    // collected results
    mutable HandleVector handles_;
  } ;
} // namespace Result

#endif // _FFT_RESULT_HPP_cm101026_
