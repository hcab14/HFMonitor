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

#include <map>
#include <boost/foreach.hpp>
#include "logging.hpp"
#include "libusb1.0/libusb1.hpp"

namespace libusb {
  // Implementation ------------------------------------------------------------
  class session_impl: public session {
  public:
    session_impl() {
      ASSERT_THROW(libusb_init(&_context) == 0);
    }
    ~session_impl() {
      libusb_exit(_context);
    }
    virtual libusb_context* get_context() const { return _context; }
    virtual void set_debug(int level) { libusb_set_debug(_context, level); }
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
      for (ssize_t i(0); i<ret; ++i) 
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
    virtual std::string get_ascii_manufacturer() const {
      return get_ascii_descriptor(this->get().iManufacturer);
    }
    virtual std::string get_ascii_product() const {
      return get_ascii_descriptor(this->get().iProduct);
    }
    virtual std::string get_ascii_serial() const {
      return get_ascii_descriptor(this->get().iSerialNumber, true);
    }
    virtual boost::uint8_t get_num_configurations() const {
      return this->get().bNumConfigurations;
    }
  private:
    std::string get_ascii_descriptor(boost::uint8_t desc_index, bool check_nonzero=false) const  {
      if (check_nonzero && desc_index == 0) return "";
      const device_handle::sptr handle(device_handle::get_cached_handle(_dev));
      unsigned char buffer[512];
      const ssize_t ret(libusb_get_string_descriptor_ascii(handle->get(),
                                                           desc_index,
                                                           buffer,
                                                           sizeof(buffer)));
      return (ret < 0) ? "" : std::string((const char *)buffer, ret);
    }
    const device::sptr _dev;
    libusb_device_descriptor _desc;
  } ;

  device_descriptor::sptr device_descriptor::make(device::sptr dev) {
    return sptr(new device_descriptor_impl(dev));
  }

  class config_descriptor_impl : public config_descriptor {
  public:
    config_descriptor_impl(device::sptr dev, boost::uint8_t configuration)
      : _config(0) {
      ASSERT_THROW(configuration < device_descriptor::make(dev)->get_num_configurations());
      ASSERT_THROW(libusb_get_config_descriptor(dev->get(), configuration, &_config) == 0);
    }    
    virtual ~config_descriptor_impl() {
      libusb_free_config_descriptor(_config);
    }

    virtual const libusb_config_descriptor* get() const { return _config; }
    virtual boost::uint8_t get_num_interfaces() const { return this->get()->bNumInterfaces; }
    virtual boost::uint8_t get_max_power() const { return this->get()->MaxPower;  }
    virtual const libusb_interface& get_interface(boost::uint8_t index) const {
      ASSERT_THROW(index < this->get_num_interfaces());
      return this->get()->interface[index];
      ;
    }
  private:
    libusb_config_descriptor* _config;
  } ;

  config_descriptor::sptr config_descriptor::make(device::sptr dev, boost::uint8_t configuration) {
    return config_descriptor::sptr(new config_descriptor_impl(dev, configuration));
  }

  class device_handle_impl : public device_handle {
  public:
    device_handle_impl(device::sptr dev)
      : _dev(dev) {
      ASSERT_THROW(libusb_open(_dev->get(), &_handle) == 0);
    }
    ~device_handle_impl() {
      BOOST_FOREACH(int claimed_interface, _claimed)
        libusb_release_interface(this->get(), claimed_interface);
      libusb_close(_handle);
    }

    virtual libusb_device_handle *get() const { return _handle; }

    virtual void detach_all_kernel_drivers() {
      for (int interface(0); interface < 4; ++interface)
        if (kernel_driver_active(interface))
          detach_kernel_driver(interface);
    }
    virtual bool kernel_driver_active(int interface) {
      int result(0);
      ASSERT_THROW((result=libusb_kernel_driver_active(this->get(), interface) >= 0));
      return result==1;
    }
    virtual void detach_kernel_driver(int interface) {
      ASSERT_THROW(libusb_detach_kernel_driver(this->get(), interface) == 0);
    }
    virtual void set_configuration(int configuration) {
      ASSERT_THROW(libusb_set_configuration(this->get(), configuration) == 0);
    }
    virtual void claim_interface(int interface) {
      ASSERT_THROW(libusb_claim_interface(this->get(), interface) == 0);
      _claimed.push_back(interface);
    }
    virtual void set_interface_alt_setting(int interface, int alternate_setting) {
      ASSERT_THROW(libusb_set_interface_alt_setting(this->get(), interface, alternate_setting) == 0);
    }
    virtual void clear_halt(unsigned char endpoint) {
      const int rc(libusb_clear_halt(this->get(), endpoint));
      ASSERT_THROW(rc == 0 || rc == LIBUSB_ERROR_NOT_FOUND);
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
      LOG_ERROR("USB open failed: see the application notes for your device.");
      throw std::runtime_error(e.what());
    }
  }

  class special_handle_impl : public special_handle {
  public:
    special_handle_impl(device::sptr dev)
      : _dev(dev) {}
    
    virtual device::sptr get_device() const { return _dev; }

    std::string get_manufacturer() const {
      return device_descriptor::make(this->get_device())->get_ascii_manufacturer();
    }
    std::string get_product() const {
      return device_descriptor::make(this->get_device())->get_ascii_product();
    }
    std::string get_serial() const {
      return device_descriptor::make(this->get_device())->get_ascii_serial();
    }
    boost::uint16_t get_vendor_id() const {
      return device_descriptor::make(this->get_device())->get().idVendor;
    }
    boost::uint16_t get_product_id() const {
      return device_descriptor::make(this->get_device())->get().idProduct;
    }
    boost::uint8_t get_num_configurations() const {
      return device_descriptor::make(this->get_device())->get_num_configurations();
    }
    boost::uint8_t get_bus_number() const {
      return libusb_get_bus_number(this->get_device()->get());
    }
    boost::uint8_t get_device_address() const {
      return libusb_get_device_address(this->get_device()->get());
    }
    boost::uint8_t get_num_interfaces(boost::uint8_t configuration_index) const {
      return config_descriptor::make(this->get_device(), configuration_index)->get_num_interfaces();
    }
    boost::uint8_t get_max_power(boost::uint8_t configuration_index) const {
      return config_descriptor::make(this->get_device(), configuration_index)->get_max_power();
    }
    const libusb_interface& get_interface(boost::uint8_t configuration_index,
                                          boost::uint8_t interface_index) const {
      return config_descriptor::make(this->get_device(), configuration_index)->get_interface(interface_index);
    }

  private:
    const device::sptr _dev;
  } ;

  special_handle::sptr special_handle::make(device::sptr dev) {
    return sptr(new special_handle_impl(dev));
  }
} // namespace libusb

std::vector<usb_device_handle::sptr> 
usb_device_handle::get_device_list(boost::uint16_t vid, boost::uint16_t pid) {
  std::vector<usb_device_handle::sptr> handles;
  const libusb::device_list::sptr dev_list(libusb::device_list::make());
  for (size_t i(0); i<dev_list->size(); ++i) {
    usb_device_handle::sptr handle(libusb::special_handle::make(dev_list->at(i)));
    if ((vid  == 0 || handle->get_vendor_id()  == vid) &&
        (pid  == 0 || handle->get_product_id() == pid))
      handles.push_back(handle);
  }    
  return handles;
}

class usb_bulk_transfer_impl : public usb_bulk_transfer {
public:
  usb_bulk_transfer_impl(libusb::device_handle::sptr handle,
                         boost::uint8_t              endpoint,
                         boost::uint16_t             length,
                         usb_transfer_callback::sptr callback,
                         unsigned int                timeout)
    : _transfer(libusb_alloc_transfer(0))
    , _buffer(0)
    , _callback(callback) { 
    ASSERT_THROW(_transfer == 0);
    _buffer.resize(length);
    libusb_fill_bulk_transfer(_transfer,
                              handle->get(),
                              endpoint,
                              &_buffer.front(),
                              _buffer.size(),
                              libusb_transfer_cb_fn(&transfer_callback),
                              this,
                              timeout);
  }

  virtual ~usb_bulk_transfer_impl() {
    libusb_free_transfer(_transfer);
  }
  virtual boost::uint8_t endpoint() const { return _transfer->endpoint; }

  virtual libusb_transfer* get() { return _transfer; }
  virtual int submit() {
    return libusb_submit_transfer(_transfer);
  }
  virtual int cancel() {
    return libusb_cancel_transfer(_transfer);
  }
private:
  static void transfer_callback(libusb_transfer* t) {
    usb_bulk_transfer_impl* ubti(reinterpret_cast<usb_bulk_transfer_impl*>(t->user_data));
    ubti->_callback->callback(t->status,
                              t->length,
                              t->actual_length,
                              t->buffer,
                              ubti);
  }

  libusb_transfer*            _transfer;
  std::vector<unsigned char>  _buffer;
  usb_transfer_callback::sptr _callback;
} ;

class libusb_control_impl : public usb_control {
public:
  libusb_control_impl(libusb::device_handle::sptr handle, int configuration)
    : _handle(handle) {
    _handle->set_configuration(configuration);
    // _handle->detach_all_kernel_drivers();
    const int control_interface_number(0);
    _handle->claim_interface(control_interface_number);
    _handle->set_interface_alt_setting(control_interface_number, 0);
  }
  virtual void clear_halt(unsigned char endpoint) {
    _handle->clear_halt(endpoint);
  }
  virtual ssize_t submit_control(boost::uint8_t  request_type,
                                 boost::uint8_t  request,
                                 boost::uint16_t value,
                                 boost::uint16_t index,
                                 unsigned char*  buff,
                                 boost::uint16_t length,
                                 unsigned int    timeout) {
    return libusb_control_transfer(_handle->get(),
                                   request_type, request, value, index, buff, length, 
                                   timeout);
  }
  virtual ssize_t submit_bulk(boost::uint8_t  endpoint,
                              boost::uint8_t* data,
                              int             length,
                              int*            transferred,
                              unsigned int    timeout) {
    return libusb_bulk_transfer(_handle->get(),
                                endpoint, data, length, transferred,
                                timeout);
  }

private:
  const libusb::device_handle::sptr _handle;
} ;


usb_control::sptr usb_control::make(usb_device_handle::sptr handle, int configuration) {
  return sptr(new libusb_control_impl
              (libusb::device_handle::get_cached_handle
               (boost::static_pointer_cast<libusb::special_handle>(handle)->get_device()), configuration));
}
