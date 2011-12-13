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
#ifndef _fifi_control_hpp_cm110516_
#define _fifi_control_hpp_cm110516_
#include "libusb1.hpp"
#include "util.hpp"

namespace FiFiSDR {
  class receiver_control {
  public:
    typedef boost::shared_ptr<receiver_control> sptr;
    class __attribute__((__packed__)) presel_entry {
    public:
      presel_entry() : freq1_(0), freq2_(0), pattern_(0) {}
      presel_entry(double freq1, double freq2, boost::uint8_t pattern);
      double freq1() const; // in MHz
      double freq2() const; // in MHz
      double pattern() const { return pattern_; }

      friend bool operator==(const presel_entry& pe1, const presel_entry& pe2) {
        return (pe1.freq1()   == pe2.freq1() &&
                pe1.freq2()   == pe2.freq2() &&
                pe1.pattern() == pe2.pattern());
      }          
      friend std::ostream& operator<<(std::ostream& os, const presel_entry& pent);
    private:
      boost::uint32_t freq1_; // counter
      boost::uint32_t freq2_; // counter
      boost::uint8_t  pattern_;
    } ;

    class si570_registers {
    public:
    private:
      boost::uint8_t regs_[6];
    } ;
    class si570_virtual_registers {
    public:
    private:
      boost::uint8_t regs_[6];
    } ;

    virtual ~receiver_control() {}

    static sptr make(usb_device_handle::sptr device);

    virtual boost::uint32_t get_version_number() const = 0;
    virtual std::string get_version_string() const = 0;

    virtual double get_frequency() const    = 0;
    virtual void set_frequency(double freq) = 0;

    virtual double get_startup_frequency() const    = 0;
    virtual void set_startup_frequency(double freq) = 0;

    virtual double get_xtal_frequency() const    = 0;
    virtual void set_xtal_frequency(double freq) = 0;

    // n == 3 || n == 5
    virtual double get_nth_frequency(unsigned n) const      = 0;
    virtual void set_nth_frequency(unsigned n, double freq)  = 0;

    virtual boost::uint32_t get_presel_mode() const   = 0;
    virtual void set_presel_mode(boost::uint32_t mode) = 0;

    virtual presel_entry get_presel_entry(size_t index) const        = 0;
    virtual void set_presel_entry(size_t index, const presel_entry&) = 0;

    virtual boost::uint8_t get_i2c_addr() const = 0;
    // virtual void set_i2c_addr(boost::uint8_t)   = 0;

    virtual boost::uint32_t get_virtual_vco_factor() const = 0;
    virtual void set_virtual_vco_factor(boost::uint32_t)   = 0;

    virtual si570_registers get_si570_registers() const = 0;

    virtual si570_virtual_registers get_si570_virtual_registers() const           = 0;
    virtual void set_si570_virtual_registers(const si570_virtual_registers& regs) = 0;

    virtual void set_abpf(boost::uint32_t index, boost::uint32_t value) = 0;

    virtual bool   have_abpf() const = 0;
    virtual bool   abpf_enabled() const = 0;
    virtual size_t num_abpf() const = 0;

  protected:
  private:
  } ;
}
#endif // _fifi_control_hpp_cm110516_
