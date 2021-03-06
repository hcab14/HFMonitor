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
#ifndef _BROADCASTER_DIRECTORY_HPP_cm111222_
#define _BROADCASTER_DIRECTORY_HPP_cm111222_

#include <map>
#include <string>
#include <sstream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>
#include <boost/noncopyable.hpp>
#include <boost/regex.hpp>
#include <boost/shared_ptr.hpp>

#include "logging.hpp"
#include "network/protocol.hpp"

namespace network {
  namespace broadcaster {
    // -----------------------------------------------------------------------------
    // broadcaster directory
    //  * hold a set of data streams
    class directory {
    public:
      typedef boost::posix_time::ptime ptime;
      typedef boost::shared_ptr<directory> sptr;
      typedef std::string index_type;
      typedef std::map<index_type, boost::uint32_t> directory_map;  // name -> unique stream number
      typedef directory_map::const_iterator const_iterator;

      // returns new stream id and a flag indicating if a new entry has been added
      std::pair<boost::uint32_t, bool>
      insert(index_type name) {
        static boost::uint32_t unique_stream_number(1); // stream number 0 -> directory
        const_iterator i(directory_.find(name));
        if (i != end()) return std::make_pair(i->second, false);
        directory_.insert(std::make_pair(name, unique_stream_number));
        return std::make_pair(unique_stream_number++, true);
      }
      void insert(index_type name,
                  boost::uint32_t unique_stream_number) {
        assert(contains(name) == false);
        directory_.insert(std::make_pair(name, unique_stream_number));
      }

      static std::string id() { return "DIR_0000"; }

      std::string serialize(ptime t) const {
        std::string bytes_dir;
        for (auto const& d : directory_)
          bytes_dir += protocol::directory_entry::serialize(d.second, d.first);
        return bytes_dir;
      }

      void update(size_t n, const char* bytes) {
        directory_.clear();
        size_t i(0);
        while (i < n) {
          const protocol::directory_entry* de(reinterpret_cast<const protocol::directory_entry* >(bytes+i));
          const std::string name(bytes+i+protocol::directory_entry::name_offset(),
                                 bytes+i+protocol::directory_entry::name_offset() + de->length_of_name());
          LOG_INFO(str(boost::format("directory::update (%04d,%d)") % de->stream_number() % name));
          directory_.insert(std::make_pair(name, de->stream_number()));
          i += de->size();
        }
        assert(i == n);
      }

      bool contains(index_type name) const {
        return directory_.find(name) != end();
      }
      bool contains(const boost::regex& e) const {
        for (auto const& d : directory_)
          if (regex_match(d.first, e)) return true;
        return regex_match("", e);
      }

      boost::uint32_t stream_number_of(index_type name) const {
        const_iterator i(directory_.find(name));
        assert(i != end());
        return i->second;
      }

      std::string stream_name_of(boost::uint32_t number) const {
        for (auto const& d : directory_)
          if (d.second == number) return d.first;
        throw std::runtime_error("no stream associated with this stream number");
      }

      std::string ls() const {
        std::ostringstream oss;
        for (auto const& d : directory_)
          oss << d.first << " " << d.second << "\n";
        oss << "\r\n";
        return oss.str();
      }

      friend std::ostream& operator<<(std::ostream& os, const directory& d) {
        for (auto i(d.begin()), iend(d.end()); i!=iend && os; ++i)
          os << i->first << " (" << i->second << ") ";
        return os;
      }

      const_iterator begin() const { return directory_.begin(); }
      const_iterator end()   const { return directory_.end(); }
    protected:

    private:
      directory_map directory_;
    } ;

  } // namespace broadcaster
} // namespace network
#endif // _BROADCASTER_DIRECTORY_HPP_cm111222_
