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
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>

#include <boost/log/core.hpp>

#include <boost/log/attributes/clock.hpp>

#include <boost/log/expressions.hpp>
#include <boost/log/expressions/formatters/date_time.hpp>

#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>

#include <boost/log/support/date_time.hpp>

#include "logging.hpp"

namespace logging {
  // Attribute value tag type
  struct severity_tag;

  // The operator is used when putting the severity level to log
  boost::log::formatting_ostream& operator<<(boost::log::formatting_ostream& strm,
					     boost::log::to_log_manip< severity_level, severity_tag > const& manip) {
    static const char* strings[] = { "I", "S", "W", "E"};  
    severity_level level(manip.get());
    if (static_cast< std::size_t >(level) < sizeof(strings) / sizeof(*strings))
      strm << strings[level];
    else
      strm << static_cast< int >(level);

    return strm;
  }
  
  BOOST_LOG_GLOBAL_LOGGER_INIT(logging::logger, severity_timestamp_logger) {
    severity_timestamp_logger<logging::severity_level> lg;
    return lg;
  }

  void init(std::string path, std::string name) {
    boost::shared_ptr< boost::log::sinks::text_file_backend > backend
    (boost::make_shared< boost::log::sinks::text_file_backend >
     (boost::log::keywords::file_name = str(boost::format("%s/%s_y%%Y-m%%m-d%%d.%%02N.log") % path % name),
      boost::log::keywords::rotation_size = 10 * 1024 * 1024,
      boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0, 0, 0)));

    backend->auto_flush(true);

    typedef boost::log::sinks::synchronous_sink< boost::log::sinks::text_file_backend > sink_t;
    boost::shared_ptr< sink_t > sink(new sink_t(backend));

    boost::log::core::get()->add_sink(sink);

    sink->set_formatter
      (boost::log::expressions::stream
       << boost::log::expressions::attr<severity_level, severity_tag>("Severity") << "-"
       << boost::log::expressions::format_date_time<boost::posix_time::ptime>("TimeTag", "[%Y-%m-%d %H:%M:%S.%f]: ")
       << boost::log::expressions::smessage
       );
  }

} // namespace logging
