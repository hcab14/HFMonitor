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
#ifndef _FFT_RESULT_HPP_cm101026_
#define _FFT_RESULT_HPP_cm101026_

#include <string>
#include <sstream>
#include <fstream>
#include <deque>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/weak_ptr.hpp>

#include "network/protocol.hpp"
#include "gen_filename.hpp"
#include "processor/result.hpp"
#include "station_info.hpp"

namespace Result {
  class Base : public gen_filename, public processor::result_base {
  public:
    typedef boost::shared_ptr<Base> sptr;
    typedef boost::shared_ptr<Base> Handle;
    typedef boost::weak_ptr<Base> WeakHandle;
    typedef std::deque<Handle> HandleVector;
    typedef std::complex<float> complex_type;

    Base(std::string name, ptime time)
      : processor::result_base(name, time)
      , posEndTime_(0) {}
    virtual ~Base() {}

    virtual std::string toString() const { return name(); }

    ptime time() const { return approx_ptime(); }
    virtual void updateTime(ptime t) {} // is overwritten by Result::Calibration

    void push_back(const Handle& h) { handles_.push_back(h); }

    const Base* getLatest() const { 
      return handles_.empty() ? this : handles_.back().get();
    }

    void clear() {
      handles_.clear();
    }

    // dump data
    void dumpToFile(std::string path,
                    std::string tag,
                    std::string s) const {
      const station_info si(s, lineBreak());
      boost::filesystem::fstream *ofs(0);
      ofs = dumpSingle(ofs, path, tag, this, si);
      BOOST_FOREACH(const HandleVector::value_type& h, handles_)
        ofs = dumpSingle(ofs, path, tag, h.get(), si);
      updateTimeTag(*ofs, posEndTime_, getLatest()->makeTimeLabel());
      delete ofs;
    }

    template<typename BROADCASTER>
    void dumpToBC(std::string path,
                  std::string tag,
                  boost::shared_ptr<BROADCASTER> bc,
                  std::string s) {
      const station_info si(s, lineBreak());
      dumpToBCSingle(path, tag, bc, this, si);
      BOOST_FOREACH(const HandleVector::value_type& h, handles_)
        dumpToBCSingle(path, tag, bc, h.get(), si);
//       for (HandleVector::const_iterator i(handles_.begin()), iend(handles_.end()); i!=iend; ++i)
//         dumpToBCSingle(path, tag, bc, i->get(), si);
    }

    template<typename BROADCASTER>
    void dumpToBCSingle(std::string path,
                        std::string tag,
                        boost::shared_ptr<BROADCASTER> bc,
                        Base* h,
                        const station_info& si) {
      std::ostringstream sHeader;
      sHeader << si;
      h->dump_header(sHeader);
      const std::string sh(sHeader.str());

      std::ostringstream sData;
      h->dump_data(sData);
      const std::string sd(sData.str());

      bc->bc_data(h->time(), tag, h->format(), sd, sh);
    }

    boost::filesystem::fstream* dumpSingle(boost::filesystem::fstream* ofs, 
                                           std::string path, std::string tag, 
                                           const Base* h,
                                           const station_info& si) const {
      const boost::filesystem::path p(gen_file_path(path, tag, h->time()));
      const bool file_exists(boost::filesystem::exists(p));
      if (not file_exists) {
        if (ofs != NULL) { 
          delete ofs; ofs= NULL; 
        }
        boost::filesystem::fstream lofs(p, std::ios::out);  
        lofs << si;
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
    virtual std::ostream& dump_header(std::ostream& os) const {
      return os << "# Time_UTC ";
    }
    virtual std::ostream& dump_data(std::ostream& os) const {
      return os;
    }

    // file context dump:
    virtual boost::filesystem::fstream& dumpHeaderToFile(boost::filesystem::fstream& os) const {
      std::string timeLabel(makeTimeLabel());
      os << "# StartTime = " << timeLabel << " [UTC]" << lineBreak();
      posEndTime_   = os.tellg() + std::streamoff(14);          
      os << "# EndTime   = " << timeLabel << " [UTC]" << lineBreak();
      std::ostringstream oss; dump_header(oss);
      os << oss.str();
      return os;
    }
    virtual boost::filesystem::fstream& dumpDataToFile(boost::filesystem::fstream& os) const {
      os << makeTimeLabel() << " ";
      std::ostringstream oss; dump_data(oss);
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
//     std::string name_;
//     ptime time_;
  private:
    std::string makeTimeLabel() const {
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%S.%f")));
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
