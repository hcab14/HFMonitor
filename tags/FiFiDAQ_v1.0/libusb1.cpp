// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

#include "util.hpp"
#include "libusb1.hpp"

const int libusb_timeout = 0;

namespace libusb {
  // Implementation ------------------------------------------------------------
  class session_impl: public session {
  public:
    session_impl() { ASSERT_THROW(libusb_init(&_context) == 0); }
    ~session_impl() { libusb_exit(_context); }
    virtual libusb_context* get_context() const { return _context; }
  private:
    libusb_context* _context;
  } ;
  
  session::sptr session::get_global_session() {
    static boost::weak_ptr<session> global_session;
    if (not global_session.expired()) return global_session.lock();
    const sptr new_global_session(new session_impl());
    global_session = new_global_session;
    return new_global_session;
  }

  class device_impl : public device {
  public:
    device_impl(libusb_device* dev) 
      : _session(session::get_global_session())
      , _dev(dev) {}
    ~device_impl() { libusb_unref_device(this->get()); }

    virtual libusb_device* get() const { return _dev; }   
  private:
    const session::sptr  _session;
    libusb_device* _dev;
  } ;

  class device_list_impl : public device_list {
  public:
    device_list_impl() {
      const session::sptr s(session::get_global_session());
      libusb_device** dev_list;
      const ssize_t ret(libusb_get_device_list(s->get_context(), &dev_list));
      if (ret < 0) 
        throw std::runtime_error("cannot enumerate usb devices");      
      for (size_t i(0); i<size_t(ret); ++i) 
        _devs.push_back(device::sptr(new device_impl(dev_list[i])));      
      libusb_free_device_list(dev_list, false/*dont unref*/);
    }
    virtual size_t size() const { return _devs.size(); }
    virtual device::sptr at(size_t i) const { return _devs.at(i); }
  private:
    std::vector<device::sptr> _devs;    
  } ;

  device_list::sptr device_list::make() {
    return sptr(new device_list_impl());
  }

  class device_descriptor_impl : public device_descriptor {
  public:
    device_descriptor_impl(device::sptr dev) 
      : _dev(dev) {
      ASSERT_THROW(libusb_get_device_descriptor(_dev->get(), &_desc) == 0);
    }
    virtual const libusb_device_descriptor& get() const { return _desc; }
    virtual std::string get_ascii_serial() const {
      if (this->get().iSerialNumber == 0) return "";
      const device_handle::sptr handle(device_handle::get_cached_handle(_dev));      
      unsigned char buff[512];
      const ssize_t ret(libusb_get_string_descriptor_ascii(handle->get(), 
                                                           this->get().iSerialNumber, 
                                                           buff, 
                                                           sizeof(buff)));
      return (ret < 0) ? "" : std::string((char *)buff, ret);
    }
  private:
    const device::sptr _dev;
    libusb_device_descriptor _desc;
  } ;

  device_descriptor::sptr device_descriptor::make(device::sptr dev) {
    return sptr(new device_descriptor_impl(dev));
  }

  class device_handle_impl : public device_handle {
  public:
    device_handle_impl(device::sptr dev)
      : _dev(dev) {
      ASSERT_THROW(libusb_open(_dev->get(), &_handle) == 0);
    }
    ~device_handle_impl() {
      for (size_t i(0); i<_claimed.size(); ++i)
        libusb_release_interface(this->get(), _claimed[i]);
      libusb_close(_handle);
    }

    virtual libusb_device_handle *get() const { return _handle; }
    virtual void claim_interface(int interface) {
      ASSERT_THROW(libusb_claim_interface(this->get(), interface) == 0);
      _claimed.push_back(interface);
    }
  private:
    const device::sptr _dev;
    libusb_device_handle* _handle;
    std::vector<int> _claimed;
  } ;

  device_handle::sptr device_handle::get_cached_handle(device::sptr dev) {
    static std::map<libusb_device*, boost::weak_ptr<device_handle> > handles;
    if (handles.find(dev->get()) != handles.end() and not handles[dev->get()].expired())
        return handles[dev->get()].lock();
    try {
        const sptr new_handle(new device_handle_impl(dev));
        handles[dev->get()] = new_handle;
        return new_handle;
    } catch (const std::exception &e) {
      std::cerr << "USB open failed: see the application notes for your device." << std::endl;
      throw std::runtime_error(e.what());
    }
  }

  class special_handle_impl : public special_handle {
  public:
    special_handle_impl(device::sptr dev)
      : _dev(dev) {}
    
    virtual device::sptr get_device() const { return _dev; }
    std::string get_serial() const {
      return device_descriptor::make(this->get_device())->get_ascii_serial();
    }
    boost::uint16_t get_vendor_id() const {
      return device_descriptor::make(this->get_device())->get().idVendor;
    }
    boost::uint16_t get_product_id() const {
      return device_descriptor::make(this->get_device())->get().idProduct;
    }
  private:
    const device::sptr _dev;
  } ;

  special_handle::sptr special_handle::make(device::sptr dev) {
    return sptr(new special_handle_impl(dev));
  }
} // namespace libusb

class libusb_special_handle_impl : public libusb::special_handle {
public:
  libusb_special_handle_impl(libusb::device::sptr dev)
    : _dev(dev) {}
  virtual libusb::device::sptr get_device() const { return _dev; }
  virtual std::string get_serial() const { 
    return libusb::device_descriptor::make(this->get_device())->get_ascii_serial();
  }
  virtual boost::uint16_t get_vendor_id() const { 
    return libusb::device_descriptor::make(this->get_device())->get().idVendor;
  }
  virtual boost::uint16_t get_product_id() const { 
    return libusb::device_descriptor::make(this->get_device())->get().idProduct;
  }  
private:
  const libusb::device::sptr _dev; 
} ;

std::vector<usb_device_handle::sptr> 
usb_device_handle::get_device_list(boost::uint16_t vid, boost::uint16_t pid) {
  std::vector<usb_device_handle::sptr> handles;
  const libusb::device_list::sptr dev_list(libusb::device_list::make());
  for (size_t i(0); i<dev_list->size(); ++i) {
    usb_device_handle::sptr handle(libusb::special_handle::make(dev_list->at(i)));
    if ((vid == 0 || handle->get_vendor_id()  == vid) &&
        (pid == 0 || handle->get_product_id() == pid))
      handles.push_back(handle);
  }    
  return handles;
}

class libusb_control_impl : public usb_control {
public:
  libusb_control_impl(libusb::device_handle::sptr handle)
    : _handle(handle) {
    _handle->claim_interface(0 /* control interface */);
  }  
  virtual ssize_t submit(boost::uint8_t  request_type,
                         boost::uint8_t  request,
                         boost::uint16_t value,
                         boost::uint16_t index,
                         unsigned char*  buff,
                         boost::uint16_t length) {
    return libusb_control_transfer(_handle->get(),
                                   request_type, request, value, index, buff, length, 
                                   libusb_timeout);
  }
private:
  const libusb::device_handle::sptr _handle;
} ;

usb_control::sptr usb_control::make(usb_device_handle::sptr handle) {
  return sptr(new libusb_control_impl
              (libusb::device_handle::get_cached_handle
               (boost::static_pointer_cast<libusb::special_handle>(handle)->get_device())));
}
