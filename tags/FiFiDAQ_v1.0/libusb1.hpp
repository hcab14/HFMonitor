// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
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

#include "util.hpp"

// Top level interface ---------------------------------------------------------
class usb_device_handle : public boost::noncopyable {
public:
  typedef boost::shared_ptr<usb_device_handle> sptr;
  virtual ~usb_device_handle() {}
  
  virtual std::string get_serial() const = 0;
  virtual boost::uint16_t get_vendor_id() const = 0;
  virtual boost::uint16_t get_product_id() const = 0;
  
  static std::vector<usb_device_handle::sptr>  
  get_device_list(boost::uint16_t vid, boost::uint16_t pid);
} ;

class usb_control : public boost::noncopyable {
public:
  typedef boost::shared_ptr<usb_control> sptr;
  virtual ~usb_control() {}

  static sptr make(usb_device_handle::sptr);
  virtual ssize_t submit(boost::uint8_t  request_type,
                         boost::uint8_t  request,
                         boost::uint16_t value,
                         boost::uint16_t index, 
                         unsigned char*  buff,
                         boost::uint16_t length) = 0;
} ;

namespace libusb {
  class session : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<session> sptr;
    virtual ~session() {}
    static sptr get_global_session();
    virtual libusb_context* get_context() const = 0;
  private:
    libusb_context* usb_context;
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
    virtual void claim_interface(int) = 0;
  } ;

  class device_descriptor : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<device_descriptor> sptr;
    virtual ~device_descriptor() {}
    static sptr make(device::sptr);

    virtual const libusb_device_descriptor& get() const = 0;
    virtual std::string get_ascii_serial() const = 0;
  } ;

  class special_handle : public usb_device_handle {
  public:
    typedef boost::shared_ptr<special_handle> sptr;
    static sptr make(device::sptr);
    virtual device::sptr get_device() const = 0;
  } ;
}
#endif // _libusb1_hpp_cm_110307_
