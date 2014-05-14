// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2013 Christoph Mayer
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
#ifndef _logging_hpp_cm101217_
#define _logging_hpp_cm101217_

#include <stdexcept>
#include <boost/current_function.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>

#include "gen_filename.hpp"

#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
              "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
              "  at " + std::string(__FILE__) + ":" + BOOST_STRINGIZE(__LINE__) + "\n")

#define ASSERT_THROW(_x)                                                \
  if (not (_x))                                                         \
    throw std::runtime_error(THROW_SITE_INFO("assertion failed: " + std::string(#_x))); \
  else void(0)

#define ASSERT_THROW_PERSEUS(_x)                                        \
  if (not (_x))                                                         \
    throw std::runtime_error(THROW_SITE_INFO("assertion failed: " + std::string(#_x) \
                                             + " msg: " + std::string(perseus_errorstr()))); \
  else void(0) 

namespace logging {
  typedef enum log_kind {
    INFO,
    STATUS,
    WARNING,
    ERROR
  } log_kind;
  
  class logger : public gen_filename {
  public:
    typedef boost::posix_time::ptime ptime;
    typedef boost::shared_ptr<logger> sptr;

    virtual ~logger() {}

    static sptr make(std::string basePath, 
                     std::string name) { 
      return sptr(new logger(basePath, name)); 
    }

    virtual file_period filePeriod() const { return gen_filename::PeriodDay; }
    virtual std::string filePrefix() const { return name_; }
    virtual std::string fileExtension() const { return "log"; }

    logger& operator<<(std::string message) {
      return this->log(log_kind_, message);
    }

    logger& log(log_kind k, std::string message) {
      const ptime now(boost::posix_time::microsec_clock::universal_time());
      return this->log(k, now, message);
    }

    logger& log(log_kind k, const ptime& now, std::string message) {
      log_kind_ = k;
      std::string type;
      switch (k) {
      case INFO:
        type="I"; break;
      case STATUS:
        type="S"; break;
      case WARNING: 
        type="W"; break;
      case ERROR:
        type="E"; break;
      default: 
        throw std::runtime_error("unknown log type");
      }
      boost::filesystem::fstream ofs(gen_file_path(basePath_, "", now), std::ios::out | std::ios::app);
      ofs << type << "-[" << now << "]: " << message << std::endl;
      return *this;
    }

    logger& set_kind(log_kind k) { 
      log_kind_=k; 
      return *this;
    }

  static sptr get(sptr l=sptr()) {
    static sptr lp(l);
    return lp;
  }

  protected:    
  private:
    logger(std::string basePath,
           std::string name) 
      : basePath_(basePath) 
      , name_(name)
      , log_kind_(ERROR) {}
    const std::string basePath_;  
    const std::string name_;
    log_kind log_kind_;    
  } ;
  
}

#define LOGGER_INIT(_path_, _name_) logging::logger::get(logging::logger::make(_path_, std::string(_name_)+"."))

#define LOG_INFO(_message_)    logging::logger::get()->log(logging::INFO,    _message_)
#define LOG_STATUS(_message_)  logging::logger::get()->log(logging::STATUS,  _message_)
#define LOG_WARNING(_message_) logging::logger::get()->log(logging::WARNING, _message_)
#define LOG_ERROR(_message_)   logging::logger::get()->log(logging::ERROR,   _message_)

#define LOG_INFO_T(_time_, _message_)    logging::logger::get()->log(logging::INFO,    _time_, _message_)
#define LOG_STATUS_T(_time_, _message_)  logging::logger::get()->log(logging::STATUS,  _time_, _message_)
#define LOG_WARNING_T(_time_, _message_) logging::logger::get()->log(logging::WARNING, _time_, _message_)
#define LOG_ERROR_T(_time_, _message_)   logging::logger::get()->log(logging::ERROR,   _time_, _message_)

#define LOGGER_INFO    logging::logger::get()->set_kind(logging::INFO)
#define LOGGER_STATUS  logging::logger::get()->set_kind(logging::STATUS)
#define LOGGER_WARNING logging::logger::get()->set_kind(logging::WARNING)
#define LOGGER_ERROR   logging::logger::get()->set_kind(logging::ERROR)

#endif // _logging_hpp_cm101217_
