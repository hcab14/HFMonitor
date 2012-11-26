// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "include/sdr/perseus/perseus_control.hpp"
#include "include/buffer.hpp"
#include "include/network/broadcaster.hpp"

#include "include/network/protocol.hpp"

// perseus callback
//  * makes copy of data, and
//  * inserts the data into a buffer
class test_cb : public Perseus::callback {
public:
  test_cb(buffer<std::string>::sptr buf)
    : buffer_(buf) {}
  virtual ~test_cb() {}
  void operator()(unsigned char* data, size_t length) {    
    std::string s((char*)(data), length);
    buffer_->insert(boost::posix_time::microsec_clock::universal_time(),
                    s);
  }
protected:
private:
  buffer<std::string>::sptr buffer_;
} ;

void bc_ttest() {
  std::cerr << "bc_ttest()" << std::endl;
}

// bridge between perseus and the broadcaster
//  * running in its own thread
//  * the data taken out of the buffer sent to the broadcaster
class bridge {
public:
  bridge(boost::asio::io_service& service,
         buffer<std::string>::sptr buffer,
         broadcaster::sptr broadcaster) 
    : buffer_(buffer)
    , broadcaster_(broadcaster)
    , strand_(service)
    , counter_(0)
    , do_run_(true)
    , mutex_(new boost::mutex)
    , cond_(new boost::condition) {}

  virtual ~bridge() {}

  void broadcast_data() {
    boost::mutex::scoped_lock lock(*mutex_);
    ++counter_;
    // std::cout << "broadcast_data " << data_.first << std::endl;
    // make up data packet
    const iq_info header_iq(125e3, 250e3, 'I', 3);
    const header header("IQ__", data_.first, sizeof(iq_info)+data_.second.size());
    std::string d;
    std::copy(header.begin(),       header.end(),       std::back_inserter(d));
    std::copy(header_iq.begin(),    header_iq.end(),    std::back_inserter(d));
    std::copy(data_.second.begin(), data_.second.end(), std::back_inserter(d));
    broadcaster_->bc_data(data_.first, "DataIQ", d);
    cond_->notify_one();
  }

  void operator()() {    
    while (do_run_) {
      try {
        boost::mutex::scoped_lock lock(*mutex_);
        data_= buffer_->get(); // this throws an exception if the buffer is stopped

        // std::cout << "get ----" << " " << data_.first << " "<< counter_ << " "
        //           << (strand_.get_io_service().stopped() ? "stopped" : "running") << std::endl;
        strand_.post(strand_.wrap(boost::bind(&bridge::broadcast_data, this)));
        do_run_ = !strand_.get_io_service().stopped();
        if (do_run_)
          cond_->wait(lock);
      } catch (...) {
        do_run_= false;
        std::cout << "************* stopped ***********" << std::endl;
      }
    }
    // finish
  }

private:

  buffer<std::string>::sptr buffer_;
  broadcaster::sptr         broadcaster_;
  boost::asio::strand       strand_;
  size_t                    counter_;
  bool                      do_run_;
  buffer<std::string>::ptime_data_pair data_; // current to be broadcaster data
  boost::shared_ptr<boost::mutex>     mutex_;              // mutex
  boost::shared_ptr<boost::condition> cond_;               // condition used to signal new data
} ;

int main(int argc, char* argv[])
{
  boost::asio::io_service service;
  // boost::shared_ptr<boost::asio::io_service::work> work_(new boost::asio::io_service::work(service));
#if 1
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


    if (index != -1) {
      broadcaster::sptr bc(broadcaster::make_broadcaster(service, config.get_child("Broadcaster")));
      bc->start();

      // buffer between perseus and broadcaster
      buffer<std::string>::sptr buf(buffer<std::string>::make_buffer(1024*1000, boost::posix_time::seconds(1)));

      std::cout << "io_service: " << (service.stopped() ? "stopped" : "running") << std::endl;

      // set up receiver
      Perseus::callback::sptr cb(new test_cb(buf));
      Perseus::receiver_control::sptr rec(Perseus::receiver_control::make(index));
      rec->init(config_perseus);
      rec->set_sample_rate(125000);
      
      std::vector<boost::shared_ptr<boost::thread> > threads;

      const size_t thread_pool_size_(1);
      for (std::size_t i(0); i<thread_pool_size_; ++i) {
        boost::shared_ptr<boost::thread> thread
          (new boost::thread(boost::bind(&boost::asio::io_service::run, &service)));
        threads.push_back(thread);
      }

      std::cout << "io_service: " << (service.stopped() ? "stopped" : "running") << std::endl;


      // bridge //running in a thread
      bridge r(service, buf, bc);
      {
        boost::shared_ptr<boost::thread> bridge_thread(new boost::thread(r));
        threads.push_back(bridge_thread);
      }

      std::cout << "io_service: " << (service.stopped() ? "stopped" : "running") << std::endl;
      

      std::cout << "io_service: " << (service.stopped() ? "stopped" : "running") << std::endl;

      // start receiver callback
      rec->start_async_input(cb);

      std::cout << "io_service: " << (service.stopped() ? "stopped" : "running") << std::endl;
      sleep(120);

      // std::cout << "**** stop perseus *** " << std::endl;
      rec->stop_async_input();
      
      // std::cout << "**** stop buffer *** " << std::endl;
      buf->stop();

      // std::cout << "**** stop broadcaster *** " << std::endl;
      bc->stop();

      std::cout << "**** stop io service *** " << std::endl;
      service.stop();

      std::cout << "**** stop threads *** " << std::endl;
      // Wait for all threads in the pool to exit.
      for (std::size_t i(0); i<threads.size(); ++i)
        threads[i]->join();      
      std::cout << "**** stopped *** " << std::endl;
    }
    
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
#endif
  return 0;
}
