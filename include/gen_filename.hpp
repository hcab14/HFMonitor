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
#ifndef _GEN_FILENAME_HPP_cm101217_
#define _GEN_FILENAME_HPP_cm101217_

#include <string>
#include <stdexcept>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>

/// filename generation according to a time stamp
class gen_filename : private boost::noncopyable {
public:
  typedef enum file_period {
    PeriodDay,
    PeriodHour,
    Period5Minutes,
    Period1Minute
  } file_period;
  
  virtual ~gen_filename() {}

  virtual file_period filePeriod() const { return PeriodDay; }
  virtual std::string filePrefix() const { return ""; }
  virtual std::string fileExtension() const { return "txt"; }
  virtual boost::filesystem::path gen_file_path(std::string basePath,
                                                std::string tag,
                                                boost::posix_time::ptime t) const {
    boost::filesystem::path p(basePath+"/"+tag);
    boost::filesystem::create_directories(p);
    std::stringstream oss;
    // the created time_facet is owned by \c oss and therefore does not need to be destroyed
    oss.imbue(std::locale(oss.getloc(), 
                          new boost::posix_time::time_facet(make_time_format(filePeriod()).c_str())));
    // compute time modulo 5 minutes
    if (filePeriod() == Period5Minutes) {
      const boost::posix_time::time_duration td(t.time_of_day());         
      t= boost::posix_time::ptime(t.date(), 
                                  boost::posix_time::time_duration(td.hours(), 5*(td.minutes()/5), 0));
    }
    oss << "/" << filePrefix() << t << "." << fileExtension();
    return p/=(oss.str());
  }
  
  static file_period str2period(std::string s) {
    if (s == "1d") return PeriodDay;
    if (s == "1h") return PeriodHour;
    if (s == "5m") return Period5Minutes;
    if (s == "1m") return Period1Minute;
    throw std::runtime_error("str2period: invalid period '" + s + "'");
  }

protected:
private:
  static std::string make_time_format(file_period p) {
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
      throw std::runtime_error("requested file_period is not supported");
    }
    return time_facet_format;
  }
} ;

#endif // _GEN_FILENAME_HPP_cm101217_
