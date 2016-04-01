// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
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

#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/noncopyable.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#include "buffer.hpp"
#include "libusb1.0/libusb1.hpp"
#include "network.hpp"
#include "network/broadcaster.hpp"
#include "network/protocol.hpp"
#include "run.hpp"
#include "sdr/perseus/perseus_control.hpp"

/*! \addtogroup executables
 *  HFMontor Executables
 *  @{
 * \addtogroup perseus_server perseus_server
 * perseus_server
 * - controls the perseus SDR
 * - provides a datastream with I/Q on a specified TPC/IP port
 * - configration:  command line / XML file
 * 
 * @{
 */

//! Callback called by perseus sdr:
///  - makes copy of data, and
///  - inserts the data into @ref buffer
class test_cb : public Perseus::callback {
public:
  enum {
    BUFFER_SIZE = 16320*10
  };
  test_cb()
    : i_(0)
    , error_counter_(0)
    , dt_sec_expected_(0) {}
  test_cb(buffer<std::string>::sptr buf, int sample_rate)
    : i_(0)
    , error_counter_(0)
    , dt_sec_expected_(BUFFER_SIZE/6./double(sample_rate))
    , buffer_(buf) {}

  virtual ~test_cb() { }

  void operator()(unsigned char* data, size_t length) {
    if (i_ == BUFFER_SIZE) { // buffer is full
      std::string s(reinterpret_cast<char*>(data_), BUFFER_SIZE);
      buffer_->insert(t_, s);
      i_ = 0;
    }
    if (i_ == 0) {
      boost::posix_time::ptime t(boost::posix_time::microsec_clock::universal_time());
      if (!t_.is_not_a_date_time()) {
        const double dt(1e-3*(t-t_).total_milliseconds());
        if (dt < 0.8*dt_sec_expected_ || dt > 1.2*dt_sec_expected_ ) {
          LOG_ERROR(str(boost::format("test_cb: dt=%f dt_expected=%f") % dt % dt_sec_expected_));
          if (error_counter_ > 5)
            throw std::runtime_error("too low/high callback rate");
          ++error_counter_;
        } else {
          error_counter_ = 0;
        }
      }
      t_ = t;
    }
    if (i_+length > BUFFER_SIZE)
      throw std::runtime_error("invalid buffer size");

    std::copy(data, data+length, data_+i_);
    i_ += length;
  }
protected:
private:
  size_t i_;
  size_t error_counter_;
  double dt_sec_expected_; // expected time difference
  buffer<std::string>::sptr buffer_;
  boost::posix_time::ptime t_;
  unsigned char data_[BUFFER_SIZE];
} ;

//! bridge between perseus and the broadcaster
///  - runs in its own thread
///  - data is taken out of the buffer and sent to  @ref network::broadcaster::broadcaster
class bridge {
public:
  bridge(buffer<std::string>::sptr buffer,
         network::broadcaster::broadcaster::sptr broadcaster,
         Perseus::receiver_control::sptr receiver)
    : buffer_(buffer)
    , broadcaster_(broadcaster)
    , strand_(broadcaster->get_strand())
    , receiver_(receiver)
    , do_run_(true)
    , mutex_(new boost::mutex)
    , cond_(new boost::condition) 
  {}

  ~bridge() {}

  void operator()() {
    while (do_run_) {
      try {
        boost::mutex::scoped_lock lock(*mutex_);
        data_= buffer_->get(); // this throws an exception if the buffer is stopped
        strand_.dispatch(strand_.wrap(boost::bind(&bridge::broadcast_data, this)));
//         strand_.post(strand_.wrap(boost::bind(&bridge::broadcast_data, this)));
        do_run_ = !strand_.get_io_service().stopped();
        if (do_run_)
          cond_->wait(lock);
      } catch (const std::exception& e) {
        LOG_ERROR(str(boost::format("bridge::operator() stopped '%s'") % e.what()));
        do_run_= false;
      }
    }
    // finish
  }
protected:
  void broadcast_data() {
    boost::mutex::scoped_lock lock(*mutex_);
    // make up data packet
    const network::protocol::iq_info
      header_iq(receiver_->get_sample_rate(),
                receiver_->get_center_frequency_hz(),
                'I', 3, 0);
    const std::string stream_name("DataIQ");
    std::string d(sizeof(header_iq)+data_.second.size(), 0);
    std::copy(header_iq.begin(),    header_iq.end(),    d.begin());
    std::copy(data_.second.begin(), data_.second.end(), d.begin()+sizeof(header_iq));
    broadcaster_->bc_data(data_.first, stream_name, "IQ__0000", d);
    cond_->notify_one();
  }

private:
  buffer<std::string>::sptr       buffer_;
  network::broadcaster::broadcaster::sptr broadcaster_;
  boost::asio::strand&            strand_;
  Perseus::receiver_control::sptr receiver_;
  bool                            do_run_;
  buffer<std::string>::ptime_data_pair data_;   // current to be broadcaster data
  boost::shared_ptr<boost::mutex>      mutex_;  // mutex
  boost::shared_ptr<boost::condition>  cond_;   // condition used to signal new data
} ;

/// for debugging
void unexpected_handler() {
  std::cerr << "unexpected called; uncaught_exception=" << std::uncaught_exception() << std::endl;
  throw std::runtime_error("unexpected called");
}
/// for debugging
void terminate_handler() {
  std::cerr << "terminate handler called uncaught_exception=" << std::uncaught_exception() << std::endl;
  abort();
}

int main(int argc, char* argv[])
{ 
  std::set_unexpected(unexpected_handler);
  std::set_terminate(terminate_handler);

  // process command-line arguments
  boost::program_options::variables_map vm;
  try {
    vm = process_options("config/perseus_server.xml", argc, argv);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  // inititalize @ref logger.hpp
  boost::filesystem::path p(vm["config"].as<std::string>());
  LOGGER_INIT("./Log", p.stem().native());
  try {
    // read XML configuration 
    boost::property_tree::ptree config;
    read_xml(vm["config"].as<std::string>(), config, boost::property_tree::xml_parser::no_comments);

    // set up a usb session
    libusb::session::sptr s(libusb::session::get_global_session());

    const boost::property_tree::ptree& config_perseus(config.get_child("Perseus"));
    const std::string perseus_firmware(config_perseus.get<std::string>("<xmlattr>.firmware"));

    // get the number of connected perseus SDRs
    const size_t num_perseus(Perseus::receiver_control::get_num_perseus());
    std::cout << "found " << num_perseus << std::endl;

    if (num_perseus == 0)
      throw std::runtime_error("no perseus receiver found");    

    // find the right perseus SDR
    int index(-1);
    for (size_t i(0); i<num_perseus; ++i) {
      const Perseus::product_id pid(Perseus::receiver_control::get_product_id_at(i, perseus_firmware));
      std::cout << str(boost::format("index=%2d: %s") % i % pid.to_str()) << std::endl;
      if (pid.sn() == config_perseus.get<size_t>("<xmlattr>.sn", 0))
        index = i;
    }

    if (index != -1) {
      std::vector<boost::shared_ptr<boost::thread> > threads;
      network::broadcaster::broadcaster::sptr bc;
      buffer<std::string>::sptr buf;
      Perseus::receiver_control::sptr rec;
      {
        // wait until a signal is sent/service is stopped
        wait_for_signal w(network::get_io_service());
        w.add_signal(SIGINT).add_signal(SIGQUIT).add_signal(SIGTERM);        

        const boost::property_tree::ptree& config_broadcaster(config_perseus.get_child("Broadcaster"));
        bc = network::broadcaster::broadcaster::make(config_broadcaster);

        bc->start();

        // buffer between perseus and broadcaster
        buf = buffer<std::string>::make_buffer(1024*1000, boost::posix_time::seconds(1));

        // set up the receiver
        rec = Perseus::receiver_control::make(index);
        rec->init(config_perseus);
        Perseus::callback::sptr cb(new test_cb(buf, rec->get_sample_rate()));

        // thread pool setup
        const size_t thread_pool_size(config_broadcaster.get<size_t>("<xmlattr>.threadPoolSize"));
        for (std::size_t i(0); i<thread_pool_size; ++i) {
          boost::shared_ptr<boost::thread> thread
            (new boost::thread(boost::bind(&boost::asio::io_service::run, &network::get_io_service())));
          threads.push_back(thread);
        }

        // bridge running in a thread
        {
          bridge r(buf, bc, rec);
          boost::shared_ptr<boost::thread> bridge_thread(new boost::thread(r));
          threads.push_back(bridge_thread);
        }

        // start receiver callback
        rec->start_async_input(cb);
        
      } // here the destructor of wait_for_signal waits for a signal

      rec->stop_async_input();
      buf->stop();
      bc->stop();
      usleep(1000*1000);
      network::get_io_service().stop();

      // Wait for all threads in the pool to exit.
      BOOST_FOREACH(boost::shared_ptr<boost::thread> thr, threads)
        thr->join();
      
      std::cout << "**** stopped *** " << std::endl;
    }
  } catch (const std::exception &e) {
    LOG_ERROR(e.what()); 
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
/*! @} */
/*! @} */
