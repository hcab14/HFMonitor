// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _BROADCASTER_DIRECTORY_HPP_cm111222_
#define _BROADCASTER_DIRECTORY_HPP_cm111222_

#include <map>
#include <string>
#include <sstream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/regex.hpp>
#include <boost/shared_ptr.hpp>

#include "network/protocol.hpp"

// -----------------------------------------------------------------------------
// broadcaster_directory
//  * hold a set of data streams

class broadcaster_directory : private boost::noncopyable {
public:
  typedef boost::posix_time::ptime ptime;
  typedef boost::shared_ptr<broadcaster_directory> sptr;
  typedef std::string index_type;
  typedef std::map<index_type, boost::uint32_t> directory_map;  // name -> unique stream number
  typedef directory_map::const_iterator const_iterator;

  // static sptr make() { return sptr(new broadcaster_directory); }

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

  std::string serialize(ptime t) const {
    std::string bytes_dir;
    for (const_iterator i(begin()); i!=end(); ++i)
      bytes_dir += directory_entry::serialize(i->second, i->first);
    const header h("DIR_0000", t, 0, bytes_dir.size());
    std::string bytes;
    std::copy(h.begin(), h.end(), std::back_inserter(bytes));
    bytes += bytes_dir;
    return bytes;
  }

  void update(size_t n, const char* bytes) {
    directory_.clear();
    std::cout << "directory::update n=" << n << "'" << std::string(bytes,bytes+n) << "'" <<std::endl;
    size_t i(0);
    while (i < n) {
      const directory_entry* de(reinterpret_cast<const directory_entry* >(bytes+i));
      std::cout << "directory::update (" << de->length_of_name() << "," << de->stream_number() << ") " << i << " " << n << std::endl;
      const std::string name(bytes+i+directory_entry::name_offset(),
                             bytes+i+directory_entry::name_offset() + de->length_of_name());
      std::cout << "directory::update (" << name << "," << de->stream_number() << ") " << i << " " << n << std::endl;
      directory_.insert(std::make_pair(name, de->stream_number()));
      i += de->size();
    }
    assert(i == n);
  }

  bool contains(index_type name) const {
    return directory_.find(name) != end();
  }
  bool contains(const boost::regex& e) const {
    for (const_iterator i(begin()); i!=end(); ++i)
      if (regex_match(i->first, e)) return true;
    return regex_match("", e);
  }

  boost::uint32_t stream_number_of(index_type name) const {
    const_iterator i(directory_.find(name));
    assert(i != end());
    return i->second;
  }

  std::string ls() const {
    std::ostringstream oss;
    for (const_iterator i(begin()); i!=end(); ++i)
      oss << i->first << " " << i->second << "\n";
    oss << "\r\n";
    return oss.str();
  }

  friend std::ostream& operator<<(std::ostream& os, const broadcaster_directory& d) {
    for (broadcaster_directory::const_iterator i(d.begin()); i!=d.end() && os; ++i)
      os << i->first << " (" << i->second << ") ";
    return os;
  }
  
protected:
  const_iterator begin() const { return directory_.begin(); }
  const_iterator end()   const { return directory_.end(); }

private:
  directory_map directory_;
} ;

#endif // _BROADCASTER_DIRECTORY_HPP_cm111222_
