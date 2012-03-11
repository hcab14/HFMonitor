// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _BROADCASTER_DIRECTORY_HPP_cm111222_
#define _BROADCASTER_DIRECTORY_HPP_cm111222_

#include <set>
#include <string>
#include <sstream>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

// -----------------------------------------------------------------------------
// broadcaster_directory
//  * hold a set of data streams

class broadcaster_directory : private boost::noncopyable {
public:
  typedef boost::shared_ptr<broadcaster_directory> sptr;
  typedef std::string index_type;
  typedef std::set<index_type> directory_set;
  typedef directory_set::const_iterator const_iterator;

  static sptr make() { return sptr(new broadcaster_directory); }

  const_iterator begin() const { return directory_set_.begin(); }
  const_iterator end()   const { return directory_set_.end(); }

  void insert(index_type name) { directory_set_.insert(name); }

  bool contains(index_type name) const { return directory_set_.find(name) != end(); }

  std::string ls() const {
    std::ostringstream oss;
    for (const_iterator i=begin(); i!=end(); ++i)
      oss << *i << "\r\n";
    return oss.str();
  }
private:
  directory_set directory_set_;
} ;

#endif // _BROADCASTER_DIRECTORY_HPP_cm111222_
