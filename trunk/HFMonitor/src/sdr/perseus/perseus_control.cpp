// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2011 Christoph Mayer
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
#include <vector>
#include <numeric>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include "sdr/perseus/perseus_control.hpp"
#include "sdr/perseus/fx2_control.hpp"
#include "sdr/perseus/input_queue.hpp"

namespace Perseus {
  class receiver_control_impl : public receiver_control {
  public:
    enum {
      _vendor_id  = 0x04B4,
      _product_id = 0x325C
    } ;
    
    receiver_control_impl(size_t index)
    : _fx2_control(fx2_control::make(_vendor_id, _product_id, index))
    , _frontend_ctl(0xFF)
    , _use_preselector(false)
    , _filter_cutoffs(make_filter_cutoffs()) {
      if (_poll_libusb_thread == 0) {
        _poll_libusb_refcount = 1;
        ASSERT_THROW(pthread_create(&_poll_libusb_thread, NULL, 
                                    poll_libusb_thread_fn, NULL) == 0);
      } else {
        ++_poll_libusb_refcount;
      }
    }
    virtual ~receiver_control_impl() {
      std::cerr << "~receiver_control_impl" << std::endl;
      _input_queue.reset();
      if (--_poll_libusb_refcount == 0) {
        std::cerr << "receiver_control_impl join ..." << std::endl;
        pthread_join(_poll_libusb_thread, NULL);
        std::cerr << "receiver_control_impl joined" << std::endl;
        _poll_libusb_thread = 0;
      }
    }

    virtual void init(const boost::property_tree::ptree& config) {
      std::cout << "init:" << std::endl;        
      
      // set_attenuator(0); sleep(1);
      // set_attenuator(1); sleep(1);
      // set_attenuator(2); sleep(1);
      // set_attenuator(3); sleep(1);
      // set_attenuator(2); sleep(1);
      // set_attenuator(1); sleep(1);
      // set_attenuator(0); sleep(1);

      _fx2_control->load_fpga("perseus125k.rbs");
      const double f = 5.123e6;
      set_center_freq_hz(f);
      use_preselector(false);
      std::cout << "center freq= " << get_center_frequency_hz() << " df="
                << get_center_frequency_hz()-f << std::endl;
    }

    virtual void enable_dither(bool b) {
      set_sio(b, FPGA::sioctl::CMD::dither);
    }
    virtual void enable_preamp(bool b) {
      set_sio(b, FPGA::sioctl::CMD::gain_high);
    }
    virtual void start_async_input(callback::sptr callback) {
      _input_queue = input_queue::make(callback, 510 /*16230*/, 8,
                                       libusb::device_handle::get_cached_handle
                                       (boost::static_pointer_cast<libusb::special_handle>
                                        (_fx2_control->get_usb_device_handle())->get_device()), 
                                       fx2_control::EndPoint::data_in);
      set_sio(true, FPGA::sioctl::CMD::fifo_enable);
    }
    virtual void stop_async_input() {
      _input_queue.reset();
      set_sio(false, FPGA::sioctl::CMD::fifo_enable);
    }
    virtual void use_preselector(bool b) {
      if (_use_preselector == b) return;
      set_presel_id(b ? get_preselector_id(get_center_frequency_hz()) : 10);
      _use_preselector = b;
    }
    virtual double get_center_frequency_hz() const { 
      return counts2hz(_sio_ctl.freg);
    }
    virtual double set_center_freq_hz(double center_freq_hz) {
      ASSERT_THROW(center_freq_hz >= 0.0  && center_freq_hz < 40e6);
      set_presel_id(_use_preselector ? get_preselector_id(center_freq_hz) : 10);
      _fx2_control->fpga_sio(_sio_ctl.set_freg(hz2counts(center_freq_hz)));
      return get_center_frequency_hz();
    }
    virtual void set_attenuator(boost::uint8_t atten_id) {
      ASSERT_THROW(atten_id<4);
      if (atten_id == (_frontend_ctl >> 4)) return;
      _frontend_ctl = _fx2_control->set_port((atten_id << 4) | (_frontend_ctl & 0x0F));
    }

  private:
    typedef std::vector<double> filter_cutoffs;
    static double counts2hz(boost::uint32_t counts) {
      return counts/double(1ULL<<32)*80e6;
    }
    static boost::uint32_t hz2counts(double freq_hz) {
      return boost::uint32_t(freq_hz/80e6*(1ULL<<32));
    }
    static filter_cutoffs make_filter_cutoffs() {
      const double flt[] = {1.7e6, 2.1e6, 3.0e6, 4.2e6, 6.0e6, 8.4e6, 12e6, 17e6, 24e6, 32e6};
      return filter_cutoffs(flt, flt+sizeof(flt)/sizeof(double));
    }
    void set_presel_id(boost::uint8_t presel_id) {
      if (presel_id == (_frontend_ctl & 0x0F)) return;
      _frontend_ctl = _fx2_control->set_port((_frontend_ctl & 0xF0) | (presel_id & 0x0F));
    }
    boost::uint8_t get_preselector_id(double freq_hz) const {
      std::vector<double>::const_iterator
        i(std::find_if(_filter_cutoffs.begin(),
                       _filter_cutoffs.end(),
                       std::bind1st(std::less<double>(), freq_hz)));
      return std::distance(_filter_cutoffs.begin(), i); 
    }
    void set_sio(bool b, boost::uint8_t c) {
      _fx2_control->fpga_sio(_sio_ctl.set_ctl(b ? (_sio_ctl.ctl | c) : (_sio_ctl.ctl & (~c))));
    }

    static void* poll_libusb_thread_fn(void*) {
      int maxpri(0);
      if ((maxpri = sched_get_priority_max(SCHED_FIFO))>=0) {
        struct sched_param sparam;
        sparam.sched_priority = maxpri;
        std::cerr << "setting thread priority to " << maxpri << std::endl;
        if (pthread_setschedparam(_poll_libusb_thread, SCHED_FIFO, &sparam) < 0)
          std::cerr << "pthread_setschedparam" << std::endl;
      }
      // handle libusb events until perseus_exit is called
      while (_poll_libusb_refcount > 0) {
        static struct timeval tv = {1,0};
        libusb_handle_events_timeout(NULL, &tv);
      }
      std::cerr << "poll_libusb_thread_fn: exit" << std::endl;
      return 0;
    }

    fx2_control::sptr    _fx2_control;
    FPGA::sioctl         _sio_ctl;
    boost::uint8_t       _frontend_ctl;
    bool                 _use_preselector;
    const filter_cutoffs _filter_cutoffs;
    input_queue::sptr    _input_queue;
    static pthread_t     _poll_libusb_thread;
    static int           _poll_libusb_refcount;
  } ;

  pthread_t receiver_control_impl::_poll_libusb_thread   = 0;
  int       receiver_control_impl::_poll_libusb_refcount = 0;

  size_t receiver_control::get_num_perseus() {
    return usb_device_handle::get_device_list(receiver_control_impl::_vendor_id,
                                              receiver_control_impl::_product_id).size();
  }

  product_id receiver_control::get_product_id_at(size_t index) {
    fx2_control::sptr fxc(fx2_control::make(receiver_control_impl::_vendor_id,
                                            receiver_control_impl::_product_id,
                                            index));
    return fxc->get_eeprom_pid();
  }

  receiver_control::sptr receiver_control::make(size_t index) {
    return receiver_control::sptr(new receiver_control_impl(index));
  }
} // namespace Perseus
