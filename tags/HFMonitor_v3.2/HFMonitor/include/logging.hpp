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
#ifndef _logging_boost_hpp_cm140515_
#define _logging_boost_hpp_cm140515_

#include <stdexcept>

#include <boost/mpl/quote.hpp>
#include <boost/parameter/keyword.hpp>

#include <boost/current_function.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>

#include <boost/scope_exit.hpp>

#include <boost/thread/shared_mutex.hpp>

#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>

#include <boost/log/utility/strictest_lock.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
              "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
              "  at " + std::string(__FILE__) + ":" + BOOST_STRINGIZE(__LINE__) + "\n")

#define ASSERT_THROW(_x)                                                \
  if (not (_x))                                                         \
    throw std::runtime_error(THROW_SITE_INFO("assertion failed: " + std::string(#_x))); \
  else void(0)

namespace logging {
  enum severity_level {
    info,
    status,
    warning,
    error
  };

  namespace my_keywords {
    BOOST_PARAMETER_KEYWORD(time_tag_ns, time_tag);
  }

  template< typename BaseT >
  class time_tagger_feature : public BaseT {
  public:
    typedef typename BaseT::char_type char_type;
    typedef typename BaseT::threading_model threading_model;

  public:
    time_tagger_feature();
    time_tagger_feature(time_tagger_feature const& that);
    template< typename ArgsT >
    time_tagger_feature(ArgsT const& args);

    typedef typename boost::log::strictest_lock<boost::lock_guard<threading_model>,
                                             typename BaseT::open_record_lock,
                                             typename BaseT::add_attribute_lock,
                                             typename BaseT::remove_attribute_lock
                                             >::type open_record_lock;

  protected:
    template<typename ArgsT>
      boost::log::record open_record_unlocked(ArgsT const& args);
  } ;

  template< typename BaseT >
  time_tagger_feature< BaseT >::time_tagger_feature() {}

  template< typename BaseT >
  time_tagger_feature< BaseT >::time_tagger_feature(time_tagger_feature const& that)
    : BaseT(static_cast< BaseT const& >(that)) {}

  template< typename BaseT >
  template< typename ArgsT >
  time_tagger_feature< BaseT >::time_tagger_feature(ArgsT const& args)
    : BaseT(args) {}

  template< typename BaseT >
  template< typename ArgsT >
  boost::log::record time_tagger_feature< BaseT >::open_record_unlocked(ArgsT const& args) {
    boost::posix_time::ptime tag_value = args[my_keywords::time_tag | boost::date_time::not_a_date_time];

    // if no time is given use the current universal time
    if (tag_value.is_not_a_date_time())
      tag_value = boost::posix_time::microsec_clock::universal_time();

    boost::log::attribute_set& attrs = BaseT::attributes();
    boost::log::attribute_set::iterator tag = attrs.end();

    std::pair<boost::log::attribute_set::iterator, bool>
      res = BaseT::add_attribute_unlocked("TimeTag", boost::log::attributes::constant<boost::posix_time::ptime>(tag_value));
    if (res.second)
      tag = res.first;

    BOOST_SCOPE_EXIT_TPL((&tag)(&attrs)) {
      if (tag != attrs.end())
        attrs.erase(tag);
    }
    BOOST_SCOPE_EXIT_END
      return BaseT::open_record_unlocked(args);
  }

  struct time_tagger : public boost::mpl::quote1<time_tagger_feature> {};

  /// logger class
  template<typename LevelT = int>
  class severity_timestamp_logger
    : public boost::log::sources::basic_composite_logger<
    char,
    severity_timestamp_logger<LevelT>,
    boost::log::sources::multi_thread_model<boost::shared_mutex>,
    boost::log::sources::features<boost::log::sources::severity<LevelT>, time_tagger>
    > {
    BOOST_LOG_FORWARD_LOGGER_MEMBERS_TEMPLATE(severity_timestamp_logger);
  };
  
  void init(std::string path, std::string name);

  BOOST_LOG_GLOBAL_LOGGER(logger, severity_timestamp_logger<severity_level>);

} // namespace logging


#define LOG_WITH_TIME_TAG(lg, sev, tg)					\
  BOOST_LOG_WITH_PARAMS((lg), (boost::log::keywords::severity=(sev))(logging::my_keywords::time_tag=(tg)))

#define LOG_WITHOUT_TIME_TAG(lg, sev)                                   \
  BOOST_LOG_WITH_PARAMS((lg), (boost::log::keywords::severity=(sev)))

#define LOGGER_INIT(_path_, _name_) logging::init(_path_, _name_);

#define LOG_INFO(_message_)                                             \
  LOG_WITHOUT_TIME_TAG(logging::logger::get(), logging::info)    << _message_;
#define LOG_STATUS(_message_)                                           \
  LOG_WITHOUT_TIME_TAG(logging::logger::get(), logging::status)  << _message_;
#define LOG_WARNING(_message_)                                          \
  LOG_WITHOUT_TIME_TAG(logging::logger::get(), logging::warning) << _message_;
#define LOG_ERROR(_message_)                                            \
  LOG_WITHOUT_TIME_TAG(logging::logger::get(), logging::error)   << _message_;

// log with timestamp given as argument
#define LOG_INFO_T(_time_,_message_)                                    \
  LOG_WITH_TIME_TAG(logging::logger::get(), logging::info,    _time_) << _message_;
#define LOG_STATUS_T(_time_,_message_)                                  \
  LOG_WITH_TIME_TAG(logging::logger::get(), logging::status,  _time_) << _message_;
#define LOG_WARNING_T(_time_,_message_)                                 \
  LOG_WITH_TIME_TAG(logging::logger::get(), logging::warning, _time_) << _message_;
#define LOG_ERROR_T(_time_,_message_)                                   \
  LOG_WITH_TIME_TAG(logging::logger::get(), logging::error,   _time_) << _message_;

#define LOGGER_INFO    LOG_WITHOUT_TIME_TAG(logging::logger::get(), logging::info)
#define LOGGER_STATUS  LOG_WITHOUT_TIME_TAG(logging::logger::get(), logging::status)
#define LOGGER_WARNING LOG_WITHOUT_TIME_TAG(logging::logger::get(), logging::warning)
#define LOGGER_ERROR   LOG_WITHOUT_TIME_TAG(logging::logger::get(), logging::error)

#endif // _logging_boost_hpp_cm140515_
