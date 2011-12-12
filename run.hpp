// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _run_hpp_cm100727
#define _run_hpp_cm100727

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>

#include "logging.hpp"

#include <pthread.h>
#include <signal.h>

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
    sleep(2);
#if 0
    int sig(0);
    sigwait(&_wait_mask, &sig);
#else
    const timespec ts = {1, 0}; // check every 1 second if the service is still running
    siginfo_t siginfo;
    while (sigtimedwait(&_wait_mask, &siginfo, &ts) < 0) {
      switch (errno) {
      case EAGAIN: // no signal in _wait_mask occured during time period: check if service is still alive
        if (_service.stopped()) return; 
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

  wait_for_signal& add_signal(int s) {
    sigaddset(&_wait_mask, s);
    return *this;
  }
protected:
private:
  boost::asio::io_service& _service;
  sigset_t _new_mask;
  sigset_t _old_mask;
  sigset_t _wait_mask;
} ;

template<typename T>
void run(boost::asio::io_service& io_service, T& c) {
  typedef boost::shared_ptr<boost::thread> thread_sptr;
  thread_sptr tp;
  {
    wait_for_signal s(io_service);
    s.add_signal(SIGINT)
     .add_signal(SIGQUIT)
     .add_signal(SIGTERM);
    tp = thread_sptr(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service)));
  }  
  c.stop();
  tp->join();
}

#endif // _run_hpp_cm100727
