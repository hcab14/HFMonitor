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

namespace Perseus {
  class I8HEXEntry {
  public:
    typedef enum Type {
      DataRecord=0,
      EndOfFileRecord=1
    } Type;
    typedef std::vector<boost::uint8_t> code_vector;

    I8HEXEntry(std::string line) {
      ASSERT_THROW(line[0] == ':');
      ASSERT_THROW(line.size() > 9);
      _addr = hex2int(std::string(line,3,4));
      const boost::uint8_t type(hex2int(std::string(line,7,2)));
      ASSERT_THROW(type == DataRecord || type == EndOfFileRecord);
      if (type == DataRecord)      _type = DataRecord;
      if (type == EndOfFileRecord) _type = EndOfFileRecord;
      const size_t length(hex2int(std::string(line,1,2)));
      for (size_t i=0; i<length; ++i)
        _code.push_back(hex2int(std::string(line,9+2*i,2)));
      ASSERT_THROW(hex2int(std::string(line,9+2*length,2)) == checksum());
    }
    std::string to_str() const {
      std::string line(str(boost::format(":%02X%04X%02X") 
                           % _code.size()
                           % int(_addr)
                           % _type));
      BOOST_FOREACH(boost::uint8_t c, _code)
        line += str(boost::format("%02X") % int(c));
      return line + str(boost::format("%02X") % int(checksum()));
    }

    Type get_type() const { return _type; }
    boost::uint16_t get_addr() const { return _addr; }
    const code_vector& get_code() const { return _code; }
  protected:
  private:
    boost::uint8_t checksum() const { // = 2-compement of sum of all bytes
      return 1 + (0xFF ^ (_code.size()
                          + _type
                          + (_addr&0xFF)
                          + (_addr>>8)
                          + std::accumulate(_code.begin(), _code.end(), 0)));
    }
    static unsigned int hex2int(const std::string s) {
      unsigned result(0);
      std::stringstream iss; iss << std::hex << s; iss >> result;
      return result;
    }
    boost::uint16_t _addr;
    Type            _type;
    code_vector     _code;
  } ;

  namespace EEPROM {
    struct __attribute__((__packed__)) cmd {
      boost::int8_t   op;
      boost::uint16_t addr;
      boost::uint8_t  count;
    } ;
    
    struct __attribute__((__packed__)) reply_header {
      boost::uint8_t op;
      boost::int8_t  op_retcode;
    } ;
    
    template<typename T>
    struct __attribute__((__packed__)) reply {
      reply_header   header;
      T              data;
    } ;
  } //  namespace EEPROM

  class fx2_control : private boost::noncopyable {
  public:
    fx2_control(boost::uint16_t vendor_id,
                boost::uint16_t product_id,
                size_t index)
      : _vendor_id(vendor_id)
      , _product_id(product_id)
      , _index(index)
      , _ep_cmd    (0x01)
      , _ep_status (0x81)
      , _ep_data_in(0x82)
      , _is_configured(false) {
      try {
        const std::vector<usb_device_handle::sptr>
          v(usb_device_handle::get_device_list(_vendor_id, _product_id));
        ASSERT_THROW(_index < v.size());
        _usb_control = usb_control::make(v[index]);
        
        const Perseus::product_id prodid(read_eeprom<Perseus::product_id>(8));
        _usb_control->clear_halt(_ep_cmd);
        _usb_control->clear_halt(_ep_status);
        _usb_control->clear_halt(_ep_data_in);
        _is_configured = true;
        std::cout << "device is already configured " << prodid.to_str() << std::endl;
      } catch (...) {
        // no action
      }
    }
    virtual ~fx2_control() {
      if (this->is_configured()) {
        try {
          this->shutdown();
        } catch (...) {
          std::cerr << "ERR ~fx2_control" << std::endl;
        }
      }
    }
        
    usb_control::sptr get_usb_control() { return _usb_control; }

    bool is_configured() const { return _is_configured; }
    
    void load_firmware(std::string hex_filename) {
      this->reset(true);
      std::ifstream ifs(hex_filename.c_str());
      std::string line("");
      while (ifs>>line) {
        const I8HEXEntry hex(line);
        if (hex.get_type() == I8HEXEntry::EndOfFileRecord) break;
        ASSERT_THROW(_usb_control->submit_control(0x40, 0xA0, hex.get_addr(), 0,
                                                  (unsigned char*)(&hex.get_code().front()),
                                                  hex.get_code().size(), 1000)
                     == ssize_t(hex.get_code().size()));
      }
      this->reset(false);
      _is_configured = true;

      _usb_control.reset();
      sleep(4);

      const std::vector<usb_device_handle::sptr>
        v(usb_device_handle::get_device_list(_vendor_id, _product_id));
      ASSERT_THROW(_index < v.size());
      _usb_control = usb_control::make(v.at(_index));
    }

    template<typename T>
    T read_eeprom(boost::uint16_t addr) {
      int transferred(0);
      const EEPROM::cmd cmd = {0x06, addr, sizeof(T)};
      ASSERT_THROW(_usb_control->submit_bulk(_ep_cmd,
                                             (unsigned char *)&cmd, sizeof(cmd),
                                             &transferred, 1000) == 0);
      typename EEPROM::reply<T> reply;
      ASSERT_THROW(_usb_control->submit_bulk(_ep_status,
                                             (unsigned char *)&reply, 
                                             sizeof(reply),
                                             &transferred, 1000) == 0);
      ASSERT_THROW(reply.header.op_retcode == true);
      return reply.data;
    }

  private:
    void reset(boost::uint8_t reset) {
      ASSERT_THROW(_usb_control->submit_control(0x40, 0xA0, 0xE600, 0, &reset, 1, 1000) == 1);
    }
    void shutdown() {
      int transferred(0);
      const boost::uint8_t cmd_shutdown(0x08);
      ASSERT_THROW(_usb_control->submit_bulk(_ep_cmd, (unsigned char *)&cmd_shutdown, 1,
                                             &transferred, 1000) == 0);
      ASSERT_THROW(transferred == sizeof(cmd_shutdown));
    }

    usb_control::sptr  _usb_control;
    boost::uint16_t   _vendor_id;
    boost::uint16_t   _product_id;
    size_t            _index;
    boost::uint8_t    _ep_cmd;
    boost::uint8_t    _ep_status;
    boost::uint8_t    _ep_data_in;
    bool              _is_configured;
  } ;

  class receiver_control_impl : public receiver_control {
  public:
    enum {
      _vendor_id  = 0x04B4,
      _product_id = 0x325C
    };

    receiver_control_impl(size_t index)
      : _fx2_control(_vendor_id, _product_id, index) {}
    virtual ~receiver_control_impl() {}

    virtual void init(const boost::property_tree::ptree& config) {
#if 0
      {
        const std::vector<usb_device_handle::sptr> v(usb_device_handle::get_device_list(_vendor_id, _product_id));
        BOOST_FOREACH(const usb_device_handle::sptr& device, v) {
          std::cerr << (boost::format("vendor_id=0x%04X product_id=0x%04X manufacturer='%s' product='%s' serial='%s' [%d] %02X:%02X") 
                        % device->get_vendor_id() 
                        % device->get_product_id()
                        % device->get_manufacturer()
                        % device->get_product()
                        % device->get_serial()
                        % int(device->get_num_configurations())
                        % int(device->get_bus_number())
                        % int(device->get_device_address())
                        ) << std::endl;
        }
        _usb_control = usb_control::make(v[0], 1);
        
        // libusb::session::get_global_session()->set_debug(3);
        _usb_control->clear_halt(_ep_cmd);
        _usb_control->clear_halt(_ep_status);
        _usb_control->clear_halt(_ep_data_in);
        
        // download firmware
        fx2_load_firmware("perseus.hex");
        // fx2_shutdown();
        // renumerate
        _usb_control.reset();
        sleep(4);
      }
      {
        const std::vector<usb_device_handle::sptr> v(usb_device_handle::get_device_list(_vendor_id, _product_id));
        BOOST_FOREACH(const usb_device_handle::sptr& device, v) {
          std::cerr << (boost::format("vendor_id=0x%04X product_id=0x%04X manufacturer='%s' product='%s' serial='%s' [%d] %02X:%02X") 
                        % device->get_vendor_id() 
                        % device->get_product_id()
                        % device->get_manufacturer()
                        % device->get_product()
                        % device->get_serial()
                        % int(device->get_num_configurations())
                        % int(device->get_bus_number())
                        % int(device->get_device_address())
                        ) << std::endl;
        }
        ASSERT_THROW(v.size() != 0);
        _usb_control = usb_control::make(v[0], 1);
        std::cout << "AA" << std::endl;
        product_id prodid = fx2_read_eeprom<product_id>(8);
        std::cout << "prodid: "
                  << int(prodid.sn()) << " "
                  << int(prodid.prodcode()) << " "
                  << int(prodid.hwrel()) << " "
                  << int(prodid.hwver()) << " " 
                  << prodid.signature() << std::endl;
        
        // BOOST_FOREACH(usb_device_handle::sptr d, v) {
        //   if (d->get_bus_number() ==  &&
        //       d->get_device_address() == ) {
        //   }
      // }
      }
#endif
    }

    // usb_device_handle::sptr find_perseus() const {
    //   const std::vector<usb_device_handle::sptr> 
    //     v(usb_device_handle::get_device_list(_vendor_id, _product_id));
    // }

    // virtual boost::uint32_t get_version_number() const {
    // }
    // virtual std::string get_version_string() const {
    // }    
    // virtual double get_frequency() const { 
    // }
    // virtual void set_frequency(double freq) {
    // }
  private:
    fx2_control _fx2_control;
  } ;

  size_t receiver_control::get_num_perseus() {
    return usb_device_handle::get_device_list(receiver_control_impl::_vendor_id,
                                              receiver_control_impl::_product_id).size();
  }

  product_id receiver_control::get_product_id_at(size_t index) {
    fx2_control fxc(receiver_control_impl::_vendor_id,
                    receiver_control_impl::_product_id,
                    index);
    fxc.load_firmware("perseus.hex");
    return fxc.read_eeprom<product_id>(8);
  }

  receiver_control::sptr receiver_control::make(size_t index) {
    return receiver_control::sptr(new receiver_control_impl(index));
  }
} // namespace Perseus
