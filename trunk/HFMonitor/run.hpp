// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _run_hpp_cm100727
#define _run_hpp_cm100727

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <pthread.h>
#include <signal.h>

class wait_for_signal : public boost::noncopyable {
public:
  wait_for_signal() {
    // Block all signals for background thread.
    sigfillset(&_new_mask);
    pthread_sigmask(SIG_BLOCK, &_new_mask, &_old_mask);
  }

  ~wait_for_signal() {
    // Restore previous signals.
    pthread_sigmask(SIG_SETMASK, &_old_mask, 0);

    // Wait for signal indicating time to shut down.
    sigset_t wait_mask;
    sigemptyset(&wait_mask);
    for (std::vector<int>::const_iterator i(_signals.begin()); i!=_signals.end(); ++i)
      sigaddset(&wait_mask, *i);
    pthread_sigmask(SIG_BLOCK, &wait_mask, 0);
    int sig(0);
    sigwait(&wait_mask, &sig);
  }

  wait_for_signal& add_signal(int s) {
    _signals.push_back(s);
  }
protected:
private:
  sigset_t _new_mask;
  sigset_t _old_mask;
  std::vector<int> _signals;
} ;

template<typename T>
void run(boost::asio::io_service& io_service, T& c) {
  typedef boost::shared_ptr<boost::thread> thread_sptr;
  thread_sptr tp;
  {
    wait_for_signal s;
    s.add_signal(SIGINT)
     .add_signal(SIGQUIT)
     .add_signal(SIGTERM);
    tp = thread_sptr(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service)));
  }  
  c.stop();
  tp->join();
}

#endif // _run_hpp_cm100727
