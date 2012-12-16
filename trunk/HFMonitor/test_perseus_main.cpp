// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "buffer.hpp"
#include "network/broadcaster.hpp"
#include "network/protocol.hpp"
#include "run.hpp"
#include "sdr/perseus/perseus_control.hpp"

// perseus callback
//  * makes copy of data, and
//  * inserts the data into a buffer
class test_cb : public Perseus::callback {
public:
  test_cb(buffer<std::string>::sptr buf)
    : buffer_(buf) {}
  virtual ~test_cb() {}
  void operator()(unsigned char* data, size_t length) {    
    using namespace boost::posix_time;
    std::string s((char*)(data), length);
    const ptime t(microsec_clock::universal_time());
    buffer_->insert(t, s);
  }
protected:
private:
  buffer<std::string>::sptr buffer_;
} ;

// bridge between perseus and the broadcaster
//  * running in its own thread
//  * the data taken out of the buffer sent to the broadcaster
class bridge {
public:
  bridge(boost::asio::io_service&        service,
         buffer<std::string>::sptr       buffer,
         broadcaster::sptr               broadcaster,
         Perseus::receiver_control::sptr receiver) 
    : buffer_(buffer)
    , broadcaster_(broadcaster)
    , strand_(service)
    , receiver_(receiver)
    , counter_(0)
    , do_run_(true)
    , mutex_(new boost::mutex)
    , cond_(new boost::condition) {}

  virtual ~bridge() {}

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
protected:
  void broadcast_data() {
    boost::mutex::scoped_lock lock(*mutex_);
    ++counter_;
    // make up data packet
    const iq_info header_iq(receiver_->get_sample_rate(),
                            receiver_->get_center_frequency_hz(),
                            'I', 3, 0);
    const boost::uint32_t stream_number(0);
    const header header("IQ__0000", data_.first, stream_number,
                        sizeof(iq_info)+data_.second.size());
    std::string d;
    std::copy(header.begin(),       header.end(),       std::back_inserter(d));
    std::copy(header_iq.begin(),    header_iq.end(),    std::back_inserter(d));
    std::copy(data_.second.begin(), data_.second.end(), std::back_inserter(d));
    broadcaster_->bc_data(data_.first, "DataIQ", d);
    cond_->notify_one();
  }

private:

  buffer<std::string>::sptr       buffer_;
  broadcaster::sptr               broadcaster_;
  boost::asio::strand             strand_;
  Perseus::receiver_control::sptr receiver_;
  size_t                          counter_;
  bool                            do_run_;
  buffer<std::string>::ptime_data_pair data_;   // current to be broadcaster data
  boost::shared_ptr<boost::mutex>      mutex_;  // mutex
  boost::shared_ptr<boost::condition>  cond_;   // condition used to signal new data
} ;

int main(int argc, char* argv[])
{
  boost::asio::io_service service;
  LOGGER_INIT("./Log", "test_perseus");
  try {
    libusb::session::sptr s(libusb::session::get_global_session());
    const std::string filename((argc > 1 ) ? argv[1] : "config_perseus.xml");
    boost::property_tree::ptree config;
    read_xml(filename, config);

    const boost::property_tree::ptree& config_perseus(config.get_child("Perseus"));
    const std::string perseus_firmware(config_perseus.get<std::string>("<xmlattr>.firmware"));

    const size_t num_perseus(Perseus::receiver_control::get_num_perseus());
    std::cout << "found " << num_perseus << std::endl;

    if (num_perseus == 0)
      throw std::runtime_error("no perseus receiver found");    
    
    int index(-1);
    for (size_t i(0); i<num_perseus; ++i) {
      const Perseus::product_id pid(Perseus::receiver_control::get_product_id_at(i, perseus_firmware));
      std::cout << str(boost::format("index=%2d: %s") % i % pid.to_str()) << std::endl;
      if (pid.sn() == config_perseus.get<size_t>("<xmlattr>.sn", 0))
        index = i;
    }


    if (index != -1) {
      broadcaster::sptr bc(broadcaster::make(service, config.get_child("Broadcaster")));
      bc->start();

      // buffer between perseus and broadcaster
      buffer<std::string>::sptr buf(buffer<std::string>::make_buffer(1024*1000, boost::posix_time::seconds(1)));

      std::cout << "io_service: " << (service.stopped() ? "stopped" : "running") << std::endl;

      // set up receiver
      Perseus::callback::sptr cb(new test_cb(buf));
      Perseus::receiver_control::sptr rec(Perseus::receiver_control::make(index));
      rec->init(config_perseus);

      std::vector<boost::shared_ptr<boost::thread> > threads;
      const size_t thread_pool_size_(1);
      for (std::size_t i(0); i<thread_pool_size_; ++i) {
        boost::shared_ptr<boost::thread> thread
          (new boost::thread(boost::bind(&boost::asio::io_service::run, &service)));
        threads.push_back(thread);
      }

      // bridge running in a thread
      bridge r(service, buf, bc, rec);
      {
        boost::shared_ptr<boost::thread> bridge_thread(new boost::thread(r));
        threads.push_back(bridge_thread);
      }

      // start receiver callback
      rec->start_async_input(cb);

      { // wait until a signal is sent/service is stopped
        wait_for_signal w(service);
        w.add_signal(SIGINT).add_signal(SIGQUIT).add_signal(SIGTERM);
      }

      rec->stop_async_input();      
      buf->stop();
      bc->stop();
      service.stop();

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
  return 0;
}
