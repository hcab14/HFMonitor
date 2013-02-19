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
#ifndef _BUFFER_HPP_cm1201105_
#define _BUFFER_HPP_cm1201105_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// buffer for producer/consumer applications with
//  * (almost) non-blocking insert
//  * blocking get()
//  * maximum size (time and types) of history 
//
template<typename T>
class buffer {
public: 
  typedef T data_type;
  typedef boost::shared_ptr<buffer<T> >    sptr;
  typedef boost::mutex::scoped_lock        scoped_lock;
  typedef boost::posix_time::ptime         ptime;
  typedef boost::posix_time::time_duration time_duration;
  typedef std::pair<ptime, data_type>      ptime_data_pair;
  typedef std::deque<ptime_data_pair>      list_of_packets;

  virtual ~buffer() {}

  static sptr make_buffer(size_t max_total_size,
                          time_duration max_queue_delay) {
    return sptr(new buffer(max_total_size, max_queue_delay));
  }

  // almost non-blocking insert
  void insert(ptime t, const data_type& dp) {
    scoped_lock lock(mutex_);
    // update delay (for debugging/status display only)
    max_delay_ = std::max(max_delay_, delay(t));
    // check if buffer is too full
    size_t n_omit(0);
    if (not empty() && (total_size() > max_total_size_ || front().first+max_queue_delay_ < t)) {
      for (; size() > 0; ++n_omit)
        list_of_packets_.pop_front();
    }
    // insert data
    list_of_packets_.push_back(std::make_pair(t, dp));
    cond_.notify_one();
  }

  // blocking get method
  ptime_data_pair get(const time_duration timeout=boost::posix_time::seconds(1)) {
    scoped_lock lock(mutex_);
    while (empty()) {
      cond_.timed_wait(lock, timeout);
      if (!run_) throw std::runtime_error("buffer exit");
    }
    const ptime_data_pair d(front());
    pop_front();
    return d;
  }
  
  size_t total_size() const {
    size_t sum(0);
    BOOST_FOREACH(const typename list_of_packets::value_type& lp, list_of_packets_)
      sum += lp.second.size();
    return sum;
  }
  time_duration max_delay() const { return max_delay_; }

  void reset_max_delay() {
    scoped_lock lock(mutex_);
    max_delay_ = boost::posix_time::seconds(0);
  }
  double max_delay_msec() const {
    return 1e3*max_delay().ticks() / time_duration::ticks_per_second();
  }
  void stop() {
    scoped_lock lock(mutex_);
    run_ = false;
  }
protected:
  // the two following non-const methods are supposed to be used only when protected by a mutex
  void pop_front() { list_of_packets_.pop_front(); }
  void pop_back()  { list_of_packets_.pop_back(); }
  
  typename list_of_packets::const_reference front() const { return list_of_packets_.front(); }
  typename list_of_packets::const_reference back() const { return list_of_packets_.back(); }

  bool   empty() const { return list_of_packets_.empty(); }
  size_t size()  const { return list_of_packets_.size(); }
    
  time_duration delay(ptime t) const {
    return (empty()) ? boost::posix_time::seconds(0) : t - front().first;
  }
private:
  buffer(size_t max_total_size,
         time_duration max_queue_delay)
    : max_total_size_(max_total_size)
    , max_queue_delay_(max_queue_delay)
    , max_delay_(boost::posix_time::seconds(0))
    , run_(true) {}

  buffer(const buffer<T>& );               // not implemented: only smart ptrs to be used
  buffer<T>& operator=(const buffer<T>& ); // not implemented: only smart ptrs to be used

  const size_t        max_total_size_;     // maximum total size in bytes
  const time_duration max_queue_delay_;    // maximum delay
  time_duration       max_delay_;          // current maximum delay
  list_of_packets     list_of_packets_;    // data
  boost::mutex        mutex_;              // mutex
  boost::condition    cond_;               // condition used to signal new data
  bool                run_;                // when run_==false, get() will throw an exception
} ;

#endif // _BUFFER_HPP_cm1201105_
