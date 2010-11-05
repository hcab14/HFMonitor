// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _FFT_RESULT_HPP_cm101026_
#define _FFT_RESULT_HPP_cm101026_

#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

namespace Result {
  class Base : private boost::noncopyable {
  public:
    typedef boost::shared_ptr<Base> Handle;
    typedef enum FilePeriod {
      PeriodDay,
      PeriodHour,
      Period5Minutes,
      Period1Minute
    } FilePeriod;

    Base(std::string name) 
      : name_(name) {}
    virtual ~Base() {}
    virtual std::string toString() const { return name(); }
    std::string name() const { return name_; }

    friend std::ostream& operator<<(std::ostream& os, const Base& b) {
      return os << b.toString();
    }

    void dump(std::string path,
              std::string tag,
              boost::posix_time::ptime t) const {      
      boost::filesystem::path p(getFilePath(path, tag, t));
      const bool file_exists(boost::filesystem::exists(p));
      if (not file_exists) {
        boost::filesystem::fstream ofs(p, std::ios::out);
        dumpHeader(ofs, t) << lineBreak();
      }
      boost::filesystem::fstream ofs(p, std::ios::in | std::ios::out);
      ofs.seekp(0, std::ios::end);
      dumpData(ofs, t) << lineBreak();
    }
  protected:
    virtual std::string lineBreak() const { return "\n"; }
    virtual std::string fileExtension() const { return "txt"; }
    virtual FilePeriod filePeriod() const { return PeriodDay; }
    virtual boost::filesystem::fstream& dumpHeader(boost::filesystem::fstream& os,
                                                   boost::posix_time::ptime t) const {
      os << "# Time_UTC ";
      return os;
    }
    virtual boost::filesystem::fstream& dumpData(boost::filesystem::fstream& os,
                                                 boost::posix_time::ptime t) const {
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), new boost::posix_time::time_facet("%Y%m%dT%H%M%s")));
      oss << t;
      os << oss.str() << " ";
      return os;
    }
    virtual boost::filesystem::path getFilePath(std::string basePath,
                                                std::string tag,
                                                boost::posix_time::ptime t) const {
      boost::filesystem::path p(basePath+"/"+tag);
      boost::filesystem::create_directories(p);
      std::stringstream oss;
      oss.imbue(std::locale(oss.getloc(), 
                            new boost::posix_time::time_facet(makeTimeFormat(filePeriod()).c_str())));        
      if (filePeriod() == Period5Minutes) {
        const boost::posix_time::time_duration td(t.time_of_day());         
        t= boost::posix_time::ptime(t.date(), 
                                    boost::posix_time::time_duration(td.hours(), 5*(td.minutes()/5), 0));
      }
      oss << "/" << t << "." << fileExtension();
      return p/=(oss.str());
    }

  protected:
    std::string name_;

  private:
    static std::string makeTimeFormat(FilePeriod p) {
      std::string time_facet_format;
      switch (p) {
      case PeriodDay:
        time_facet_format="y%Y-m%m-d%d"; 
       break;
      case PeriodHour:
        time_facet_format="y%Y-m%m-d%d_H%H";
        break;
      case Period5Minutes:
      case Period1Minute:
        time_facet_format="y%Y-m%m-d%d_H%HM%M";
        break;
      default:
        throw std::runtime_error("requested FilePeriod is not supported");
      }
      return time_facet_format;
    }
  } ;
} // namespace Result

#endif // _FFT_RESULT_HPP_cm101026_
