// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2016 Christoph Mayer
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
#ifndef _DECODE_RTCM2_HPP_cm160405_
#define _DECODE_RTCM2_HPP_cm160405_

#include <cmath>
#include <complex>
#include <iostream>
#include <sstream>
#include <vector>
#include <bitset>

#include <boost/property_tree/ptree.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include "logging.hpp"

namespace decode {
  class rtcm2 : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<rtcm2> sptr;    

    typedef std::vector<bool> bit_vector_type;
    typedef bit_vector_type::iterator iterator;
    typedef bit_vector_type::const_iterator const_iterator;

    union H1 {
      H1(boost::uint32_t d=0)
        : data(d) {}

      std::string to_str() const {
        return str(boost::format("stn=%04d type=%02d")
                   % stn_num()
                   % msg_type());
      }

      boost::uint32_t stn_num()  const { return fields.stn_num; }
      boost::uint32_t msg_type() const { return fields.msg_type; }
      boost::uint32_t preamble() const { return fields.preamble; }
      
      struct __attribute__((__packed__)) {
        boost::uint32_t stn_num  : 10;
        boost::uint32_t msg_type : 6;
        boost::uint32_t preamble : 8;
      } fields;
      boost::uint32_t data;
    } ;

    union H2 {
      H2(boost::uint32_t d=0)
        : data(d) {}

      std::string to_str() const {
        return str(boost::format("z_count=%04d seq=%1d num_frames=%02d")
                   % z_count()
                   % seq()
                   % num_frames());
      }

      boost::uint32_t health()     const { return fields.health;     }
      boost::uint32_t num_frames() const { return fields.num_frames; }
      boost::uint32_t seq()        const { return fields.seq;        }
      boost::uint32_t z_count()    const { return fields.z_count;    }

      struct __attribute__((__packed__)) {
        boost::uint32_t health     : 3;
        boost::uint32_t num_frames : 5;
        boost::uint32_t seq        : 3;
        boost::uint32_t z_count    : 13;
      } fields;
      boost::uint32_t data;
    } ;

    rtcm2()
      : state_(-4)
      , bit_counter_(0)
      , sync_(300)
      , sync_counter_(0)
      , sync_offset_(0)
      , frame_()
      , h1_()
      , h2_() {}

    union frame {
      frame()
        : data(0) {}

      boost::uint32_t data;
      struct {
        boost::uint32_t parity  :  6;
        boost::uint32_t payload : 24;
        boost::uint32_t b30     :  1;
        boost::uint32_t b29     :  1;
      } f;

      boost::uint8_t preamble() const {
        return ((f.payload>>16) & 0xFF);
      }

      bool check_preamble() const {
        return (preamble() == 0x66 || preamble() == 0x99);        
      }
    
      bool check_parity(boost::uint32_t &d) const {
        std::bitset<30> br(data);
      
        std::bitset<30> brr;
        std::bitset<30> bd;
        for (int i=0; i<30; ++i)
          brr[29-i] = ((data>>i) & 1);
#if 0      
        std::cout << "=== check: data= " << std::bitset<32>(data) << std::endl;
        std::cout << "=== check: payload= " << std::bitset<24>(f.payload) << std::endl;
        std::cout << "=== check: parity= " << std::bitset<6>(f.parity) << std::endl;
        std::cout << "=== check: b2930= " << f.b29 << f.b30 << " " << brr[28] << brr[29] <<  std::endl;
        std::cout << "=== check: " << ((f.payload>>16) &0xFF) << " " << ((data>>22)&0xFF) << std::endl;
        std::cout << "=== check: " << f.payload << std::endl;
        std::cout << "=== check: brr= " << brr << std::endl;
#endif
        d = 0;
        for (int i=0; i<24; ++i) {
          bd[i] = (f.b30 ^ brr[i]);
          d    |= (bd[i] << (23-i));
          // d    |= (bd[i] << (i));
        }
      
        bd[24] = f.b29 ^ bd[0] ^ bd[1] ^ bd[2] ^ bd[4] ^ bd[5] ^ bd[ 9] ^ bd[10] ^ bd[11] ^ bd[12] ^ bd[13] ^ bd[16] ^ bd[17] ^ bd[19] ^ bd[22];
        bd[25] = f.b30 ^ bd[1] ^ bd[2] ^ bd[3] ^ bd[5] ^ bd[6] ^ bd[10] ^ bd[11] ^ bd[12] ^ bd[13] ^ bd[14] ^ bd[17] ^ bd[18] ^ bd[20] ^ bd[23];
        bd[26] = f.b29 ^ bd[0] ^ bd[2] ^ bd[3] ^ bd[4] ^ bd[6] ^ bd[ 7] ^ bd[11] ^ bd[12] ^ bd[13] ^ bd[14] ^ bd[15] ^ bd[18] ^ bd[19] ^ bd[21];
        bd[27] = f.b30 ^ bd[1] ^ bd[3] ^ bd[4] ^ bd[5] ^ bd[7] ^ bd[ 8] ^ bd[12] ^ bd[13] ^ bd[14] ^ bd[15] ^ bd[16] ^ bd[19] ^ bd[20] ^ bd[22];
        bd[28] = f.b30 ^ bd[0] ^ bd[2] ^ bd[4] ^ bd[5] ^ bd[6] ^ bd[ 8] ^ bd[ 9] ^ bd[13] ^ bd[14] ^ bd[15] ^ bd[16] ^ bd[17] ^ bd[20] ^ bd[21] ^ bd[23];
        bd[29] = f.b29 ^ bd[2] ^ bd[4] ^ bd[5] ^ bd[7] ^ bd[8] ^ bd[ 9] ^ bd[10] ^ bd[12] ^ bd[14] ^ bd[18] ^ bd[21] ^ bd[22] ^ bd[23];
      
        // std::cout << "=== check: bd= " << bd << std::endl;

        int sum=0;
        for (int i=24; i<30; ++i)
          sum += (brr[i]==bd[i]);
        // std::cout << "=== check: sum=" << sum << std::endl;
        return (sum==6);
      }
    } ;

    void decode(bool b) {
      ++bit_counter_;
      if (bit_counter_ == 30)
        bit_counter_ = 0;

      frame_.data  = (frame_.data << 1);
      frame_.data |= b;

      boost::uint32_t data(0);
      sync_[sync_counter_] = frame_.check_parity(data);

      if ( (sync_counter_%30) == sync_offset_) {
        state_ = (sync_[sync_counter_] ? state_ : -3);
        switch (state_) {
        case -3:
        case -2: {
          h1_ = H1(data);
          if (frame_.check_preamble())
            state_ = -1;
          break;
        }
        case -1: {
          h2_ = H2(data);
          state_ = (h2_.num_frames() > 0 ? 0 : -3);
          break;
        }
        default:
          if (state_ == 0) {
            data_.clear();
            std::cout << h1_.to_str() << " " << h2_.to_str() << std::endl;
          }

          if (state_ >= 0) {
            data_.push_back( data     &0xFF);
            data_.push_back((data>> 8)&0xFF);
            data_.push_back((data>>16)&0xFF);
            
            ++state_;
            if (state_ == int(h2_.num_frames())) {
              decode_message();
              state_ = -3;
            }
          }
          break;
        }
      }
      
      int hist[30] = { 0 };
      for (int i=0, n=sync_.size(); i<n; ++i)
        hist[i%30] += sync_[i];

      ++sync_counter_;
      if (sync_counter_ == sync_.size()) {
        sync_counter_ = 0;
        std::cout << "sync: ";
        for (int i=0; i<30; ++i)
          std::cout << boost::format("%2d") % hist[i];
        const int idx_max = std::distance(hist, std::max_element(hist, hist+30));
        std::cout << boost::format(" | (%2d,%2d)") % idx_max % hist[idx_max] << std::endl;
        if (hist[idx_max] > 5) {
          sync_offset_ = idx_max;
          state_ = (state_ == -4 ? -3 : state_);
        } else {
          state_ = -4;
        }
      }
    }

    struct __attribute__((__packed__)) msg_1  {
      std::string to_str(int num_frames) const {
        std::string line;
        line += str(boost::format(" G%02d UDRE=%1d PRC=%+8.2f RRC=%+6.3f IOD=%3d\n")
                    % sat_id1() % udre1() % prc1() % rrc1() % iod1());
        if (num_frames >= 2)
          line += str(boost::format(" G%02d UDRE=%1d PRC=%+8.2f RRC=%+6.3f IOD=%3d\n")
                      % sat_id2() % udre2() % prc2() % rrc2() % iod2());
        if (num_frames >= 5)
          line += str(boost::format(" G%02d UDRE=%1d PRC=%+8.2f RRC=%+6.3f IOD=%3d\n")
                      % sat_id3() % udre3() % prc3() % rrc3() % iod3());
        return line;        
      }
      
      int sf1() const { return sf1_; }
      int sf2() const { return sf2_; }
      int sf3() const { return sf3_; }

      int udre1() const { return udre1_; }
      int udre2() const { return udre2_; }
      int udre3() const { return udre3_; }

      int sat_id1() const { return sat_id1_; }
      int sat_id2() const { return sat_id2_; }
      int sat_id3() const { return sat_id3_; }

      double prc1() const { double f[2] = {0.02,0.32}; return f[sf1()]*conv(prc1_, 16); }
      double prc2() const { double f[2] = {0.02,0.32}; return f[sf2()]*conv(prc2_, 16); }
      double prc3() const { double f[2] = {0.02,0.32}; return f[sf3()]*conv((prc3_up_<<8)+prc3_low_, 16); }

      double rrc1() const { double f[2] = {0.002, 0.032}; return f[sf1()]*conv(rrc1_, 8); }
      double rrc2() const { double f[2] = {0.002, 0.032}; return f[sf2()]*conv(rrc2_, 8); }
      double rrc3() const { double f[2] = {0.002, 0.032}; return f[sf3()]*conv(rrc3_, 8); }

      int iod1() const { return iod1_; }
      int iod2() const { return iod2_; }
      int iod3() const { return iod3_; }

      boost::uint32_t prc1_     : 16;
      boost::uint32_t sat_id1_  :  5;
      boost::uint32_t udre1_    :  2;
      boost::uint32_t sf1_      :  1;

      boost::uint32_t sat_id2_  :  5;
      boost::uint32_t udre2_    :  2;
      boost::uint32_t sf2_      :  1;
      boost::uint32_t iod1_     :  8;
      boost::uint32_t rrc1_     :  8;

      boost::uint32_t rrc2_     :  8;
      boost::uint32_t prc2_     : 16; 

      boost::uint32_t prc3_up_  :  8;
      boost::uint32_t sat_id3_  :  5;
      boost::uint32_t udre3_    :  2;
      boost::uint32_t sf3_      :  1;
      boost::uint32_t iod2_     :  8;

      boost::uint32_t iod3_     :  8;
      boost::uint32_t rrc3_     :  8;
      boost::uint32_t prc3_low_ :  8;
      static boost::int32_t conv(boost::uint64_t x, int bits) {
        const boost::uint64_t pow2 = (1<<(bits-1));
        return ((x & pow2) ? (-1 - ((x-pow2) ^ (pow2-1))) : x);
      }
    } ;

    struct __attribute__((__packed__)) msg_3 {
      std::string to_str() const {
        return str(boost::format("XYZ=(%10.2f,%10.2f,%10.2f) LLH=(%6.2f,%6.2f,%6.2f)")
                   % ecef_x() % ecef_y() % ecef_z()
                   % lat() % lon() % height());
      }

      double lat() const {
        return std::atan2(ecef_y(), ecef_x())*180/M_PI;
      }
      double lon() const {
        const double a(6378137.0000);
        const double b(6356752.3142);
        const double e2 ((a*a-b*b)/(a*a));
        const double ep2((a*a-b*b)/(b*b));
        const double p(std::sqrt(ecef_x()*ecef_x() + ecef_y()*ecef_y()));
        const double theta(std::atan2(a*ecef_z(), b*p));
        const double st(std::sin(theta));
        const double ct(std::cos(theta));
        return std::atan2(ecef_z() + ep2*b*st*st*st, p - e2*a*ct*ct*ct)*180/M_PI;
      }
      double height() const {
        const double a(6378137.0000);
        const double b(6356752.3142);
        const double phi(lon()/180*M_PI);
        const double cp(std::cos(phi));
        const double sp(std::sin(phi));
        const double p(std::sqrt(ecef_x()*ecef_x() + ecef_y()*ecef_y()));
        const double n(a*a/std::sqrt(a*a*cp*cp + b*b*sp*sp));
        return p/cp - n;
      }

      double ecef_x() const {
        const boost::uint32_t x(ecef_x_up_<< 8 | ecef_x_low_);
        return 0.01*conv(x, 32);
      }
      double ecef_y() const {
        const boost::uint32_t y(ecef_y_up_<<16 | ecef_y_low_);
        return 0.01*conv(y, 32);
      }
      double ecef_z() const {
        const boost::uint32_t z(ecef_z_up_<<24 | ecef_z_low_);
        return 0.01*conv(z, 32);
      }

      static boost::int32_t conv(boost::uint64_t x, int bits) {
        const boost::uint64_t pow2 = (1<<(bits-1));
        return ((x & pow2)
                ? (-1 - ((x-pow2) ^ (pow2-1)))
                : x);
      }
      boost::uint32_t ecef_x_up_  : 24;

      boost::uint32_t ecef_y_up_  : 16;
      boost::uint32_t ecef_x_low_ :  8;

      boost::uint32_t ecef_z_up_  :  8;
      boost::uint32_t ecef_y_low_ : 16;

      boost::uint32_t ecef_z_low_ : 24;
    } ;

    struct __attribute__((__packed__)) msg_7 {
      std::string to_str() const {
        return str(boost::format("LAT=%6.1f LON=%6.2f range=%.0f freq=%5.1f BCID=%04d")
                   % lat() % lon() % range() % freq() % bcid());
      }

      double lat() const {
        return 0.002747*conv(lat_, 16);
      }
      double lon() const {
        return 0.005493*conv((lon_h_<<8)+lon_l_, 16);
      }
      double range() const {
        return 1.*range_;
      }
      double freq() const {
        return 190.0 + 0.1*double((freq_h_<<6)+freq_l_);
      }
      boost::uint32_t bcid() const {
        return bcid_;
      }

      static boost::int32_t conv(boost::uint64_t x, int bits) {
        const boost::uint64_t pow2 = (1<<(bits-1));
        return ((x & pow2)
                ? (-1 - ((x-pow2) ^ (pow2-1)))
                : x);
      }
      boost::uint32_t lon_h_     :  8;
      boost::uint32_t lat_       : 16;

      boost::uint32_t freq_h_    :  6;
      boost::uint32_t range_     : 10;
      boost::uint32_t lon_l_     :  8;

      boost::uint32_t  bc_coding_ :  1;
      boost::uint32_t  sync_type_ :  1;
      boost::uint32_t  mod_code_  :  1;
      boost::uint32_t  bitrate_   :  3;

      boost::uint32_t bcid_      : 10;
      boost::uint32_t health_    :  2;
      boost::uint32_t freq_l_    :  6;
    } ;

    void decode_message() const {
      switch (h1_.msg_type()) {
      case 7: {
        for (size_t i=0; i<data_.size(); i+=9) {
          const msg_7 *m = (const msg_7*)(&data_[i]);
          std::cout << m->to_str() << std::endl;
        }
        break;
      }
      case 3: {
        const msg_3 *m = (const msg_3*)(&data_[0]);
        std::cout << m->to_str() << std::endl;        
        break;
      }
      case 1:
      case 9: {        
        for (size_t i=0; i<data_.size(); i+=15) {
          const msg_1 *m = (const msg_1*)(&data_[i]);
          std::cout << m->to_str((data_.size() - i) / 3) << std::endl;
        }
        break;
      }
      default:
        ; // NOP
      }
    }
  protected:
  private:
    int   state_;
    int   bit_counter_;

    std::vector<bool> sync_;
    size_t sync_counter_;
    size_t sync_offset_;

    frame frame_;
    H1    h1_;
    H2    h2_;
    std::vector<boost::uint8_t> data_;
  } ;
} // namespace decode

#endif // _DECODE_RTCM2_HPP_cm160405_
