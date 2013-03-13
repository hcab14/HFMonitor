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
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "sdr/perseus/perseus_control.hpp"
#include "sdr/perseus/fx2_control.hpp"
#include "sdr/perseus/I8HEXEntry.hpp"
#include "logging.hpp"

namespace Perseus {
  namespace EEPROM {
    struct cmd {
      boost::int8_t   op;
      boost::uint16_t addr;
      boost::uint8_t  count;
    } __attribute__((__packed__)) ;
    
    struct reply_header {
      boost::uint8_t op;
      boost::int8_t  op_retcode;
    } __attribute__((__packed__)) ;
    
    template<size_t N>
    struct reply {
      reply_header   header;
      boost::uint8_t data[N];
    } __attribute__((__packed__)) ;
  } //  namespace EEPROM

  namespace FPGA {
    template<typename T>
    struct sio_cmd {
      sio_cmd(boost::uint8_t c)
        : cmd(c) {}
      sio_cmd(boost::uint8_t c, const T& d)
        : cmd(c), data(d) {}
      boost::uint8_t cmd;
      T data;
    } __attribute__((__packed__));
  }


  class fx2_control_impl : public fx2_control {
  public:
    struct CMD {
      enum {
        fpga_config = 0x00,
        fpga_reset  = 0x01,
        fpga_check  = 0x02,
        fpga_sio    = 0x03,
        fx2_port    = 0x04,
        eeprom_read = 0x06,
        shutdown    = 0x08
      } ;
    } ;
    struct FX2 {
      enum {
        vendor_request = 0x40,
        firmware_load  = 0xA0,
        addr_cpucs     = 0xE600,
        usb_timeout    = 1000
      } ; 
    } ;

    fx2_control_impl(boost::uint16_t vendor_id,
                     boost::uint16_t product_id,
                     size_t index)
      : _vendor_id(vendor_id)
      , _product_id(product_id)
      , _index     (index)
      , _is_configured(false) {
      try {
        const std::vector<usb_device_handle::sptr>
          v(usb_device_handle::get_device_list(_vendor_id, _product_id));
        ASSERT_THROW(_index < v.size());
        _usb_device_handle = v[index];
        _usb_control = usb_control::make(_usb_device_handle, 1);
        _usb_control->clear_halt(EndPoint::cmd);
        _usb_control->clear_halt(EndPoint::status);
        _usb_control->clear_halt(EndPoint::data_in);
        
        _perseus_eeprom_pid = read_eeprom<Perseus::product_id>(EEPROM::ADDR::product_id);
        LOG_INFO(str(boost::format("device '%s' is already configured")
                     % _perseus_eeprom_pid.to_str()));
        _is_configured = true;
      } catch (const std::runtime_error& e) {
        LOG_ERROR(str(boost::format("fx2_control_impl::fx2_control_impl exception: '%s'") % e.what()));
        // no further action
      }
    }

    virtual ~fx2_control_impl() {
      if (is_configured()) {
        try {
          this->shutdown();
        } catch (...) {
          LOG_ERROR("fx2_control_impl::~fx2_control_impl: ERROR");
        }
      }
    }
        
    virtual usb_control::sptr get_usb_control() { return _usb_control; }
    virtual usb_device_handle::sptr get_usb_device_handle() { return _usb_device_handle; }

    virtual bool is_configured() const { return _is_configured; }
    
    virtual void load_firmware(std::string hex_filename) {
      this->reset(1);
      std::ifstream ifs(hex_filename.c_str());
      std::string line("");
      while (ifs>>line) {
        const I8HEXEntry hex(line);
        if (hex.get_type() == I8HEXEntry::EndOfFileRecord) break;
        ASSERT_THROW(_usb_control->submit_control(FX2::vendor_request,
                                                  FX2::firmware_load, hex.get_addr(), 0,
                                                  (unsigned char*)(&hex.get_code().front()),
                                                  hex.get_code().size(), FX2::usb_timeout)
                     == ssize_t(hex.get_code().size()));
      }
      this->reset(0);
      _is_configured = true;
      _usb_control.reset();
      _usb_device_handle.reset();
      LOG_INFO("USB renumeration...");
      sleep(4); // USB renumeration
      LOG_INFO("... finished");

      const std::vector<usb_device_handle::sptr>
        v(usb_device_handle::get_device_list(_vendor_id, _product_id));
      ASSERT_THROW(_index < v.size());
      _usb_device_handle = v[_index];
      _usb_control = usb_control::make(_usb_device_handle, 1);
    }

    virtual void load_fpga(std::string filename) {
      fpga_reset();
      std::ifstream ifs(filename.c_str(), std::ios::binary);
      FPGA::sio_cmd<char[63]> cmd(CMD::fpga_config);
      while (ifs.read(cmd.data, 63))
        fpga_load_line(cmd, ifs.gcount());

      fpga_load_line(cmd, ifs.gcount());
      fpga_check();
    }

    virtual boost::uint8_t set_port(boost::uint8_t port) {
      int transferred(0);
      const boost::uint8_t cmd[2] = {CMD::fx2_port, port};
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::cmd,
                                             (unsigned char *)&cmd, sizeof(cmd),
                                             &transferred, FX2::usb_timeout) == 0);
      ASSERT_THROW(transferred==sizeof(cmd));
      return port;
    }
        
    FPGA::sioctl fpga_sio(const FPGA::sioctl& sio_cmd) {
      int transferred(0);
      const FPGA::sio_cmd<FPGA::sioctl> cmd_in(CMD::fpga_sio, sio_cmd);
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::cmd,
                                             (unsigned char *)&cmd_in, sizeof(cmd_in),
                                             &transferred, FX2::usb_timeout) == 0);
      ASSERT_THROW(transferred == sizeof(cmd_in));
      FPGA::sio_cmd<FPGA::sioctl> cmd_response(CMD::fpga_sio);
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::status,
                                             (unsigned char *)&cmd_response, sizeof(cmd_response),
                                             &transferred, FX2::usb_timeout) == 0);      
      ASSERT_THROW(transferred == sizeof(cmd_response));
      LOG_INFO(str(boost::format("sio: freg=%d response.cmd=%d response.data.ctl=%d response.data.freg=%d")
                   % int(sio_cmd.freg)
                   % int(cmd_response.cmd)
                   % int(cmd_response.data.ctl)
                   % int(cmd_response.data.freg)));
      return cmd_response.data;
    }

    virtual const Perseus::product_id& get_eeprom_pid(std::string firmware_filename="perseus.hex") {
      if (not is_configured()) {
        load_firmware(firmware_filename);
        _perseus_eeprom_pid = this->read_eeprom<product_id>(EEPROM::ADDR::product_id);
      }
      return _perseus_eeprom_pid;
    }

  protected:
    virtual eeprom_data read_eeprom_lowlevel(boost::uint16_t addr, size_t length) {
      int transferred(0);
      const EEPROM::cmd cmd = {CMD::eeprom_read, addr, length};
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::cmd,
                                             (unsigned char *)&cmd, sizeof(cmd),
                                             &transferred, FX2::usb_timeout) == 0);
      const size_t max_length(10*1024);
      EEPROM::reply<max_length> reply;
      ASSERT_THROW(length < max_length);
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::status,
                                             (unsigned char *)&reply, 
                                             sizeof(EEPROM::reply_header) + length,
                                             &transferred, FX2::usb_timeout) == 0);
      ASSERT_THROW(transferred == int(sizeof(EEPROM::reply_header) + length));
      ASSERT_THROW(reply.header.op_retcode == true);
      return eeprom_data(reply.data, reply.data+length);
    }

  private:

    void fpga_load_line(const FPGA::sio_cmd<char[63]>& cmd,
                        std::streamsize gcount) {
      if (gcount == 0) return;
      int transferred(0);
      const ssize_t len(1+gcount);
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::cmd,
                                             (unsigned char *)&cmd, len,
                                             &transferred, FX2::usb_timeout) == 0);
      ASSERT_THROW(transferred==len);
    }

    void fpga_reset() {
      int transferred(0);
      const boost::uint8_t cmd(CMD::fpga_reset);
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::cmd,
                                             (unsigned char *)&cmd, sizeof(cmd),
                                             &transferred, FX2::usb_timeout) == 0);
      ASSERT_THROW(transferred == sizeof(cmd));
    }

    bool fpga_check() {
      int transferred(0);
      const boost::uint8_t cmd(CMD::fpga_check);
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::cmd,
                                             (unsigned char *)&cmd, sizeof(cmd),
                                             &transferred, FX2::usb_timeout) == 0);
      ASSERT_THROW(transferred == sizeof(cmd));

      // wait 50 ms
      const clock_t endclock(clock() + 50*CLOCKS_PER_SEC/1000);
      while (clock() < endclock) ;

      boost::uint8_t cmdcheck_status[2] = {0, 0};
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::status,
                                             (unsigned char *)&cmdcheck_status, sizeof(cmdcheck_status),
                                             &transferred, FX2::usb_timeout) == 0);
      ASSERT_THROW(transferred == sizeof(cmdcheck_status));
      if (cmdcheck_status[1] != true) {
        LOG_ERROR(str(boost::format("fpga_check error[%d,%d]")
                      % int(cmdcheck_status[0])
                      % int(cmdcheck_status[1])));
          ASSERT_THROW(cmdcheck_status[1] == true);
      }
      return true;
    }
    
    void reset(boost::uint8_t reset) {
      ASSERT_THROW(_usb_control->submit_control(FX2::vendor_request,
                                                FX2::firmware_load,
                                                FX2::addr_cpucs, 0, &reset, 1, FX2::usb_timeout) == 1);
    }
    void shutdown() {
      int transferred(0);
      const boost::uint8_t cmd_shutdown(CMD::shutdown);
      ASSERT_THROW(_usb_control->submit_bulk(EndPoint::cmd,
                                             (unsigned char *)&cmd_shutdown, sizeof(cmd_shutdown),
                                             &transferred, FX2::usb_timeout) == 0);
      ASSERT_THROW(transferred == sizeof(cmd_shutdown));
    }

    usb_control::sptr       _usb_control;
    usb_device_handle::sptr _usb_device_handle;
    boost::uint16_t     _vendor_id;
    boost::uint16_t     _product_id;
    size_t              _index;
    bool                _is_configured;
    Perseus::product_id _perseus_eeprom_pid;
  } ;

  fx2_control::sptr fx2_control::make(boost::uint16_t vendor_id,
                                      boost::uint16_t product_id,
                                      size_t index) {
    return fx2_control::sptr(new fx2_control_impl(vendor_id, product_id, index));
  }
} // namespace Perseus
