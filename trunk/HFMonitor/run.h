// -*- C++ -*-
// $Id$
#ifndef _run_h_cm100727
#define _run_h_cm100727

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <pthread.h>
#include <signal.h>

class WaitForSignal {
public:
  WaitForSignal() {
    // Block all signals for background thread.
    sigfillset(&new_mask);
    pthread_sigmask(SIG_BLOCK, &new_mask, &old_mask);
  }

  ~WaitForSignal() {
    // Restore previous signals.
    pthread_sigmask(SIG_SETMASK, &old_mask, 0);

    // Wait for signal indicating time to shut down.
    sigset_t wait_mask;
    sigemptyset(&wait_mask);
    sigaddset(&wait_mask, SIGINT);
    sigaddset(&wait_mask, SIGQUIT);
    sigaddset(&wait_mask, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &wait_mask, 0);
    int sig = 0;
    sigwait(&wait_mask, &sig);    
  }
protected:
private:
    sigset_t new_mask;
    sigset_t old_mask;
} ;

template<typename T>
void run(boost::asio::io_service& io_service, T& c)
{
 // Block all signals for background thread.
    sigset_t new_mask;
    sigfillset(&new_mask);
    sigset_t old_mask;
    pthread_sigmask(SIG_BLOCK, &new_mask, &old_mask);

    boost::thread client_thread(boost::bind(&boost::asio::io_service::run, &io_service)); 
    
    // Restore previous signals.
    pthread_sigmask(SIG_SETMASK, &old_mask, 0);

    // Wait for signal indicating time to shut down.
    sigset_t wait_mask;
    sigemptyset(&wait_mask);
    sigaddset(&wait_mask, SIGINT);
    sigaddset(&wait_mask, SIGQUIT);
    sigaddset(&wait_mask, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &wait_mask, 0);
    int sig = 0;
    sigwait(&wait_mask, &sig);

    c.stop();
    client_thread.join();
}

#endif // _run_h_cm100727
