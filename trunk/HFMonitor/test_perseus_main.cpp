// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "include/sdr/perseus/perseus_control.hpp"

class buffer {
public:
  typedef boost::mutex::scoped_lock scoped_lock;

  typedef boost::shared_ptr<buffer> sptr;
  typedef boost::posix_time::ptime ptime;
  typedef boost::posix_time::time_duration time_duration;
  typedef std::string data_type;
  typedef boost::shared_ptr<data_type> data_ptr;
  typedef std::pair<ptime, data_type> ptime_data_pair;
  typedef std::deque<ptime_data_pair> list_of_packets;

  buffer(size_t max_total_size,
         time_duration max_queue_delay)
    : max_total_size_(max_total_size)
    , max_queue_delay_(max_queue_delay)
    , max_delay_(boost::posix_time::seconds(0))
    , run_(true) {}

  ~buffer() {}

  void push_back(ptime t, const data_type& dp) {
    scoped_lock lock(mutex_);
    // std::cout << "put() " << size() << std::endl;
    // update delay
    max_delay_ = std::max(max_delay_, delay(t));
    // check if buffer is too full
    size_t n_omit(0);
    if (not empty() && (total_size() > max_total_size_ || front().first+max_queue_delay_ < t)) {
      for (; size() > 1; ++n_omit)
        list_of_packets_.pop_back();
    }
    // insert data
    list_of_packets_.push_back(std::make_pair(t, dp));
    cond_.notify_one();
  }

  ptime_data_pair get() {
    scoped_lock lock(mutex_);
    while (empty()) {
      cond_.timed_wait(lock,  boost::posix_time::seconds(1));      
      if (!run_) throw std::runtime_error("buffer exit");
    }
    ptime_data_pair d(front());
    pop_front();
    return d;
  }
  
  size_t total_size() const {
    size_t sum(0);
    BOOST_FOREACH(const list_of_packets::value_type& lp, list_of_packets_)
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
    run_ = false;
  }
protected:
  void pop_front() { list_of_packets_.pop_front(); }
  void pop_back()  { list_of_packets_.pop_back(); }
  
  list_of_packets::const_reference front() const { return list_of_packets_.front(); }
  list_of_packets::const_reference back() const { return list_of_packets_.back(); }

  bool   empty() const { return list_of_packets_.empty(); }
  size_t size()  const { return list_of_packets_.size(); }
    
  time_duration delay(ptime t) const {
    return (empty()) ? boost::posix_time::seconds(0) : t - front().first;
  }
private:
  const size_t             max_total_size_;
  const time_duration      max_queue_delay_;
  time_duration            max_delay_;
  list_of_packets          list_of_packets_;
  boost::mutex             mutex_;
  boost::condition         cond_;
  bool                     run_;
} ;

class test_cb : public Perseus::callback {
public:
  test_cb(buffer::sptr b)
    : b_(b) {}
  virtual ~test_cb() {}
  void operator()(unsigned char* data, size_t length) {    
    const boost::posix_time::ptime t = boost::posix_time::microsec_clock::universal_time();
    b_->push_back(t, std::string((char*)(data),length));
  }
protected:
private:
  buffer::sptr b_;
} ;

class reader {
public:
  reader(buffer::sptr b) 
    : b_(b) {}

  void operator()() {
    buffer::ptime_data_pair dp;
    bool do_run = true;
    while (do_run) {
      try {
        dp = b_->get();
        std::cout << "get" << " " << dp.first << std::endl;
      } catch (...) {
        do_run= false;
      }
    }
    // finish
    std::cout << "************* stopped ***********" << std::endl;
  }
private:
  buffer::sptr b_;
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", "test_perseus");
  try {
    libusb::session::sptr s = libusb::session::get_global_session();
    const std::string filename((argc > 1 ) ? argv[1] : "config_perseus.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);
    const boost::property_tree::ptree& config_perseus(config.get_child("Perseus"));

    const size_t num_perseus = Perseus::receiver_control::get_num_perseus();
    std::cout << "found " << num_perseus << std::endl;

    if (num_perseus == 0)
      throw std::runtime_error("no perseus receiver found");    
    
    int index(-1);
    for (size_t i=0; i<num_perseus; ++i) {
      const Perseus::product_id pid(Perseus::receiver_control::get_product_id_at(i));
      std::cout << str(boost::format("index=%2d: %s") % i % pid.to_str()) << std::endl;
      if (pid.sn() == config_perseus.get<size_t>("<xmlattr>.sn", 0))
        index = i;
    }

    buffer::sptr b = buffer::sptr(new buffer(1024*1000, boost::posix_time::seconds(1)));

    if (index != -1) {
      reader r(b);
      boost::thread thrd1(r);

      Perseus::callback::sptr cb(new test_cb(b));
      Perseus::receiver_control::sptr rec(Perseus::receiver_control::make(index));
      rec->init(config_perseus);
      rec->set_sample_rate(125000);
      rec->start_async_input(cb);
      sleep(2);
      std::cout << "**** stop perseus *** " << std::endl;
      rec->stop_async_input();
      b->stop();
      std::cout << "**** stop reader *** " << std::endl;
      thrd1.join();
      std::cout << "**** joined *** " << std::endl;
    }
    
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
