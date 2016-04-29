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
#ifndef _run_hpp_cm100727
#define _run_hpp_cm100727

#include <pthread.h>
#include <signal.h>

#include <stdexcept>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include "logging.hpp"
#include "process.hpp"

/// sets up command-line options
boost::program_options::variables_map
process_options(std::string default_config_file,
                int argc, char* argv[]) {
  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,?", "produce help message")
    ("version,v", "display version")
    ("config,c", po::value<std::string>()->default_value(default_config_file),
     "path to XML configuration file");
  
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::ostringstream oss;
      oss << desc;
      throw std::runtime_error(oss.str());
    }
    if (vm.count("version")) {
      std::ostringstream oss;
      oss << SVN_VERSION_STRING;
      throw std::runtime_error(oss.str());
    }
  } catch (const std::exception &e) {
    std::ostringstream oss;
    oss << e.what() << std::endl;
    if (not (vm.count("help") ||
             vm.count("version")))
      oss << desc;
    throw std::runtime_error(oss.str());
  }
  return vm;
}

/// waits for a signal in the destructor
class wait_for_signal : public boost::noncopyable {
public:
  wait_for_signal(boost::asio::io_service& service)
    : _service(service) {
    // Block all signals for background thread.
    sigfillset(&_new_mask);
    pthread_sigmask(SIG_BLOCK, &_new_mask, &_old_mask);
    sigemptyset(&_wait_mask);
  }

  ~wait_for_signal() {
    // Restore previous signals.
    pthread_sigmask(SIG_SETMASK, &_old_mask, 0);

    // Wait for signal indicating time to shut down.
    pthread_sigmask(SIG_BLOCK, &_wait_mask, 0);

#ifdef __APPLE__
    int sig(0);
    sigwait(&_wait_mask, &sig);
#else
    const timespec ts = { 1, 0 }; // check every 1 second if the service is still running
    siginfo_t siginfo;
    while (sigtimedwait(&_wait_mask, &siginfo, &ts) < 0) {
      switch (errno) {
      case EAGAIN: // no signal in _wait_mask occured during time period: check if service is still alive
        if (_service.stopped()) {
          std::cout << "_service.stopped()\n";
          return; 
        }
        BOOST_FOREACH(process::sptr p, _processes) {
          if (!p || !p->is_running())
            return;
        }
      case EINTR: // interrupted by a signal other than in _wait_mask: ignore
        break;
      case EINVAL: // invalid timeout
        THROW_SITE_INFO("sigtimedwait: invalid timeout");
        break;
      default: // invalid errno
        THROW_SITE_INFO(str(boost::format("sigtimedwait: invalid errno %d") % errno));
      }
    }
#endif
  }

  boost::asio::io_service& get_service() { return _service; }
  
  wait_for_signal& add_signal(int s) {
    sigaddset(&_wait_mask, s);
    return *this;
  }
  wait_for_signal& add_process(process::sptr p) {
    _processes.push_back(p);
    return *this;
  }

protected:
private:
  boost::asio::io_service& _service;
  sigset_t _new_mask;
  sigset_t _old_mask;
  sigset_t _wait_mask;
  std::vector<process::sptr> _processes;
} ;

/// returns a reference to the thread pool (singleton)
boost::thread_group& get_thread_pool() {
  static boost::thread_group threadpool;
  return threadpool;
}

/// run the \c io_service in a backgroud thread until
///  - it is stopped
///  - a signal is caught
void run_in_thread(boost::asio::io_service& io_service) {
  typedef boost::shared_ptr<boost::thread> thread_sptr;
  thread_sptr tp;
  {
    wait_for_signal s(io_service);
    s.add_signal(SIGINT).add_signal(SIGQUIT).add_signal(SIGTERM);
    tp = thread_sptr(new boost::thread
                     (boost::bind
                      (&boost::asio::io_service::run,
                       &io_service)));
  }
  io_service.stop();
  tp->join();
}

#endif // _run_hpp_cm100727
