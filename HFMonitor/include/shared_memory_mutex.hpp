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
#ifndef _SHARED_MEMORY_LOCK_HPP_cm140322_
#define _SHARED_MEMORY_LOCK_HPP_cm140322_

#include <stdexcept>
#include <string>

#include <boost/format.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_upgradable_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/upgradable_lock.hpp>

class shared_data {
public:
  typedef boost::interprocess::interprocess_upgradable_mutex mutex_type;

  const mutex_type& get_mutex() const { return mutex_; }
  mutex_type& get_mutex() { return mutex_; }

protected:
private:
  mutable mutex_type mutex_;
} ;

class shm_manager_parent {
public:
  shm_manager_parent(std::string name="SO12439099-MySharedMemory")
    : name_(remove_shm(name))
    , shm_(boost::interprocess::create_only, name.c_str(), boost::interprocess::read_write)
    , dummy_(truncate(shm_))
    , region_(shm_, boost::interprocess::read_write) {    
    // Construct the shared_data.
    new (region_.get_address()) shared_data;
  }
  ~shm_manager_parent() {
    std::cout << "~shm_manager_parent" << std::endl;
    boost::interprocess::shared_memory_object::remove(name_.c_str());
  }

  shared_data& get_shared_data() {
    return *static_cast<shared_data *>(region_.get_address());
  }
protected:
  static std::string remove_shm(std::string name) {
    boost::interprocess::shared_memory_object::remove(name.c_str());
    return name;
  }
  static int truncate(boost::interprocess::shared_memory_object& shm) {
     shm.truncate(sizeof(shared_data));
    return 0;
  }
private:
  const std::string name_;
  boost::interprocess::shared_memory_object shm_;
  const int dummy_;
  boost::interprocess::mapped_region region_;
} ;

class shm_manager_child {
public:
  shm_manager_child(const char* name="SO12439099-MySharedMemory")
    : name_(name)
    , shm_(boost::interprocess::open_only, name, boost::interprocess::read_write)
    , region_(shm_, boost::interprocess::read_write)
  {}

  ~shm_manager_child() {}

  shared_data& get_shared_data() {
    return *static_cast<shared_data *>(region_.get_address());
  }
protected:
private:
  std::string name_;
  boost::interprocess::shared_memory_object shm_;
  boost::interprocess::mapped_region        region_;
} ;

class cuda_mutex {
public:
  typedef shared_data::mutex_type mutex_type;

  static const char* name() { return "CUDA_mutex"; }

  static mutex_type& get() {
    static shm_manager_child s(cuda_mutex::name());
    return s.get_shared_data().get_mutex();
  }
} ;

#endif // _SHARED_MEMORY_LOCK_HPP_cm140322_
