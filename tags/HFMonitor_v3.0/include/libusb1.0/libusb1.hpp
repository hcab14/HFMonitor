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

#ifndef _libusb1_hpp_cm_110307_
#define _libusb1_hpp_cm_110307_
#include <iostream>

#include <libusb-1.0/libusb.h>
#include <vector>
#include <map>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/cstdint.hpp>

#include "logging.hpp"

// Top level interface ---------------------------------------------------------
class usb_device_handle : public boost::noncopyable {
public:
  typedef boost::shared_ptr<usb_device_handle> sptr;
  virtual ~usb_device_handle() {}
  
  virtual std::string get_manufacturer() const = 0;
  virtual std::string get_product() const = 0;
  virtual std::string get_serial() const = 0;
  virtual boost::uint16_t get_vendor_id() const = 0;
  virtual boost::uint16_t get_product_id() const = 0;
  virtual boost::uint8_t get_num_configurations() const = 0;
  virtual boost::uint8_t get_bus_number() const = 0;
  virtual boost::uint8_t get_device_address() const = 0;
  virtual boost::uint8_t get_num_interfaces(boost::uint8_t) const = 0;
  virtual boost::uint8_t get_max_power(boost::uint8_t) const = 0;
  virtual const libusb_interface& get_interface(boost::uint8_t, boost::uint8_t) const = 0;

  static std::vector<usb_device_handle::sptr>  
  get_device_list(boost::uint16_t vid, boost::uint16_t pid);
} ;

class usb_bulk_transfer;

class usb_transfer_callback : public boost::noncopyable {
public:
  typedef boost::shared_ptr<usb_transfer_callback> sptr;
  virtual ~usb_transfer_callback() {}

  virtual void callback(libusb_transfer_status status,
                        int                    length,
                        int                    actual_length,
                        unsigned char*         buffer,
                        usb_bulk_transfer*     transfer) = 0;
} ;

class usb_bulk_transfer : public boost::noncopyable {
public:
  typedef boost::shared_ptr<usb_bulk_transfer> sptr;
  virtual ~usb_bulk_transfer() {}

  virtual libusb_transfer* get() = 0;
  virtual int submit() = 0;
  virtual int cancel() = 0;
} ;


class usb_control : public boost::noncopyable {
public:
  typedef boost::shared_ptr<usb_control> sptr;
  virtual ~usb_control() {}

  static sptr make(usb_device_handle::sptr, int configuration=0);

  virtual void clear_halt(unsigned char) = 0;
  virtual ssize_t submit_control(boost::uint8_t  request_type,
                                 boost::uint8_t  request,
                                 boost::uint16_t value,
                                 boost::uint16_t index, 
                                 unsigned char*  buff,
                                 boost::uint16_t length,
                                 unsigned int  timeout=0) = 0;

  virtual ssize_t submit_bulk(boost::uint8_t  endpoint,
                              boost::uint8_t* data,
                              int             length,
                              int*            transferred,
                              unsigned int    timeout=0) = 0;
} ;

namespace libusb {
  class session : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<session> sptr;
    virtual ~session() {}
    static sptr get_global_session();
    virtual libusb_context* get_context() const = 0;
    virtual void set_debug(int level) = 0;
  } ;

  class device : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<device> sptr;
    virtual ~device() {}
    virtual libusb_device* get() const = 0;
  } ;

  class device_list : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<device_list> sptr;
    virtual ~device_list() {}
    static sptr make();
    virtual size_t size() const = 0;
    virtual device::sptr at(size_t index) const = 0;
  } ;
  
  class device_handle : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<device_handle> sptr;
    virtual ~device_handle() {}
    static sptr get_cached_handle(device::sptr);
    virtual libusb_device_handle* get() const = 0;

    virtual void detach_all_kernel_drivers() = 0;
    virtual bool kernel_driver_active(int) = 0;
    virtual void detach_kernel_driver(int) = 0;
    virtual void set_configuration(int) = 0;
    virtual void claim_interface(int) = 0;
    virtual void set_interface_alt_setting(int, int) = 0;
    virtual void clear_halt(unsigned char) = 0;
  } ;

  class device_descriptor : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<device_descriptor> sptr;
    virtual ~device_descriptor() {}
    static sptr make(device::sptr);

    virtual const libusb_device_descriptor& get() const = 0;
    virtual std::string    get_ascii_manufacturer() const = 0;
    virtual std::string    get_ascii_product() const = 0;
    virtual std::string    get_ascii_serial() const = 0;
    virtual boost::uint8_t get_num_configurations() const = 0;
  } ;

  class config_descriptor : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<config_descriptor> sptr;
    virtual ~config_descriptor() {}
    static sptr make(device::sptr, boost::uint8_t);

    virtual const libusb_config_descriptor* get() const = 0;
    virtual boost::uint8_t get_num_interfaces() const = 0;        
    virtual boost::uint8_t get_max_power() const = 0;
    virtual const libusb_interface& get_interface(boost::uint8_t) const = 0;
  } ;

  class special_handle : public usb_device_handle {
  public:
    typedef boost::shared_ptr<special_handle> sptr;
    static sptr make(device::sptr);
    virtual device::sptr get_device() const = 0;
  } ;
}
#endif // _libusb1_hpp_cm_110307_
