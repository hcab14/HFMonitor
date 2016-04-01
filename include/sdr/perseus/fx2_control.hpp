// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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
#ifndef _fx2_control_hpp_cm120321_
#define _fx2_control_hpp_cm120321_

#include <vector>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include "libusb1.0/libusb1.hpp"

namespace Perseus {
  namespace EEPROM {
    struct ADDR {
      enum {
        product_id = 8
      } ;
    } ;
  }
  namespace FPGA {
    // Fx2 MCU <-> FPGA serial io data structure
    struct sioctl {
      struct CMD {
        enum {
          fifo_enable = 0x01,
          dither      = 0x02,
          gain_high   = 0x04
        } ;
      } ;
      sioctl(boost::uint8_t ctl_=0, boost::uint32_t freg_=0)
        : ctl(ctl_)
        , freg(freg_) {}
      const sioctl& set_ctl(boost::uint8_t ctl_) { ctl=ctl_; return *this; }
      const sioctl& set_freg(boost::uint32_t freg_) { freg=freg_; return *this; }
      boost::uint8_t  ctl;
      boost::uint32_t freg;
    } __attribute__((__packed__));
  } // namespace FPGA

  // forward declaration
  class product_id;

  class fx2_control : private boost::noncopyable {
  public:
    struct EndPoint {    
      enum {
        cmd     = 0x01,
        status  = 0x81,
        data_in = 0x82
      } ;
    } ;
    typedef boost::shared_ptr<fx2_control> sptr;
    virtual ~fx2_control() {}
    static sptr make(boost::uint16_t, boost::uint16_t, size_t);
    
    virtual usb_control::sptr get_usb_control() = 0;
    virtual usb_device_handle::sptr get_usb_device_handle() = 0;

    virtual bool is_configured() const = 0;

    virtual void load_firmware(std::string) = 0;
    virtual void load_fpga(std::string) = 0;
    virtual boost::uint8_t set_port(boost::uint8_t) = 0;
    virtual FPGA::sioctl fpga_sio(const FPGA::sioctl&) = 0;

    template<typename T> 
    T read_eeprom(boost::uint16_t addr) {
      T result;
      const eeprom_data data(read_eeprom_lowlevel(addr, sizeof(T)));
      std::copy(data.begin(), data.end(), (boost::uint8_t*)&result);
      return result;
    }
    virtual const Perseus::product_id& get_eeprom_pid(std::string ) = 0;
  protected:
    typedef std::vector<boost::uint8_t> eeprom_data;
    virtual eeprom_data read_eeprom_lowlevel(boost::uint16_t, uint8_t) = 0;
  } ;
} // namespace Perseus
#endif //  _fx2_control_hpp_cm120321_
