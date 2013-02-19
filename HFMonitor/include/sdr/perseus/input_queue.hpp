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
#ifndef _input_queue_hpp_cm120404_
#define _input_queue_hpp_cm120404_

#include <vector>
#include <numeric>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include "sdr/perseus/perseus_control.hpp"

namespace Perseus {
  class buffer_area {
  public:
    buffer_area(size_t size, size_t length)
      : _size(size)
      , _length(length)
      , _buf(size*length, 0) {}    
    size_t size() const { return _size; }
    size_t length() const { return _length; }    
    unsigned char* get_buffer_at(size_t i) { return &_buf.front() + i*length(); }
  private:
    size_t _size;
    size_t _length;
    std::vector<unsigned char> _buf;
  } ;  

  class input_queue : private boost::noncopyable {
  private:
    //
    class transfer_data : private boost::noncopyable {
    public:
      typedef boost::shared_ptr<transfer_data> sptr;
      transfer_data(libusb_transfer* t_,
                    size_t           idx_,
                    bool             cancelled_,
                    input_queue*     iq_)
        : t(t_)
        , idx(idx_)
        , cancelled(cancelled_)
        , iq(iq_) {}

      static sptr make(libusb_transfer* t_,
                       size_t           idx_,
                       bool             cancelled_,
                       input_queue*     iq_) {
        return sptr(new transfer_data(t_,idx_,cancelled_,iq_));
      }

      libusb_transfer* t;
      size_t           idx;
      bool             cancelled;
      input_queue*     iq;
    } ;    
    typedef std::vector<transfer_data::sptr> queue;

  public:
    typedef boost::shared_ptr<input_queue> sptr;

    input_queue(callback::sptr cb,
                size_t size,
                size_t length,
                libusb::device_handle::sptr device_handle,
                boost::uint8_t endpoint)
      : _cb(cb)
      , _buf(size, length)
      , _idx_expected(0)
      , _cancelling(false)
      , _completed(false) {
      for (size_t i=0; i<length; ++i) {
        transfer_data::sptr td(transfer_data::make(libusb_alloc_transfer(0), i, false, this));
        ASSERT_THROW(td->t != 0);
        _queue.push_back(td);
        libusb_fill_bulk_transfer(_queue.back()->t,
                                  device_handle->get(),
                                  endpoint,
                                  _buf.get_buffer_at(i),
                                  _buf.size(),
                                  libusb_transfer_cb_fn(transfer_callback),
                                  td.get(),
                                  80*length);
      }
      BOOST_FOREACH(transfer_data::sptr& td, _queue)
        libusb_submit_transfer(td->t);
    }

    bool check_completed() {
      BOOST_FOREACH(const transfer_data::sptr& td, _queue) {
        // std::cerr << "check_completed " << (td->cancelled ? "cancelled" : "ok") << std::endl;
        if (not td->cancelled) return false;
      }
      _completed= true;
      return _completed;
    }
    bool is_cancelling() const { return _cancelling; }
    bool is_completed() const { return _completed; }

    virtual ~input_queue() {
      _cancelling= true;
      BOOST_FOREACH(transfer_data::sptr& td, _queue)
        libusb_cancel_transfer(td->t);
      // wait up to 5 seconds until everything is cancelled
      std::cerr << "wait_cancel: ";
      for (size_t i=0; i<50 && not is_completed(); ++i) {
        std::cerr << "." << std::flush;
        usleep(100*1000);
      }
      std::cerr << " cancelled" << std::endl;
      while (not is_completed()) ;
      BOOST_FOREACH(transfer_data::sptr& td, _queue)
        libusb_free_transfer(td->t);
      std::cerr << "~input_queue end" << std::endl;
    }

    static sptr make(callback::sptr cb,
                     size_t size,
                     size_t length,
                     libusb::device_handle::sptr device_handle,
                     boost::uint8_t endpoint) {
      return sptr(new input_queue(cb, size, length, device_handle, endpoint));
    }

    static void transfer_callback(libusb_transfer *transfer) {
      transfer_data* td = (transfer_data*)(transfer->user_data);
#if 0
      std::cerr << "transfer_callback " << int(transfer) << " " << int(transfer->user_data) 
                << " status=" << int(transfer->status) << " idx=" 
                << td->idx << " idx_expected="
                << td->iq->idx_expected() << " "
                << transfer->actual_length << " " << transfer->length
                << std::endl;
#endif
      if (td->iq->is_cancelling()) {
        td->cancelled = true;
        td->iq->check_completed();
        return;
      }
      switch (transfer->status) {
      case LIBUSB_TRANSFER_COMPLETED: {
        if (td->idx == td->iq->idx_expected()) {
          if (transfer->actual_length == transfer->length) {
            if (td->iq->_cb)
              td->iq->_cb->operator()(transfer->buffer, transfer->length);
          }
        }
      } break;
      case LIBUSB_TRANSFER_TIMED_OUT:
        break; // ->resubmit
      case LIBUSB_TRANSFER_NO_DEVICE:
        td->iq->_cancelling= true;
      default:
        td->cancelled = true;
        td->iq->check_completed();
        return;
      }      
      td->iq->increment_idx_expected();
      libusb_submit_transfer(transfer);
    }

    size_t idx_expected() const { return _idx_expected; }
    size_t increment_idx_expected() {
      _idx_expected = ((1+_idx_expected) % _buf.length());
      return _idx_expected;
    }

  protected:
  private:
    callback::sptr _cb;
    buffer_area    _buf;
    queue          _queue;
    size_t         _idx_expected;
    bool           _cancelling;
    bool           _completed;
  } ;
} // namespace Perseus
#endif // _input_queue_hpp_cm120404_
