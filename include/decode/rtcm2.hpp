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

    struct __attribute__((__packed__)) H1 {
      size_t stn_num : 10;
      size_t msg_type : 6;
      size_t preamble : 8;
    } ;
    struct __attribute__((__packed__)) H2 {
      size_t health  : 3;
      size_t n       : 5;
      size_t seq     : 3;
      size_t z_count : 13;
    } ;

    rtcm2()
      : state_(-4)
      , bit_counter_(0)
      , sync_(300)
      , sync_counter_(0)
      , sync_offset_(0)
      , n_data_words_(0)
      , frame_()
      , msg_type_(0)
      , stn_num_(0)
      , z_count_(0)
      , seq_(-1)
      , num_(0) {}

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

      boost::uint32_t data = 0;
      sync_[sync_counter_] = frame_.check_parity(data);

      if ( (sync_counter_%30) == sync_offset_) {
        state_ = (sync_[sync_counter_] ? state_ : -3);
        switch (state_) {
        case -3:
        case -2: {
          if (frame_.check_preamble()) {
            const H1* h1 = (const H1*)&data;
            msg_type_ = ((data>>10) & 0x3F);
            stn_num_  = (data & 0x3FF);
            std::cout << "     H1 type=" << int(msg_type_) << " stn=" << stn_num_ 
                      << "(" << h1->stn_num << " " << h1->msg_type << ")" 
                      << std::endl;
            state_ = -1;
          }
          break;
        }
        case -1: {
          const H2* h2 = (const H2*)&data;
          z_count_ = ((data>>11) & 0x1FFF);
          seq_     = ((data>> 8) & 0x7);
          num_     = ((data>> 3) & 0x1F);
          std::cout << "     H2 z_count=" << z_count_ << " seq=" << int(seq_) << " num_frames=" << int(num_)
                    << "(" << h2->z_count << " " << h2->seq << ")" 
                    << std::endl;
          n_data_words_ = num_;
          state_ = (num_ > 0 ? 0 : -3);
          break;
        }
        default:
          if (state_ == 0)
            data_.clear();

          if (state_ >= 0) {
            data_.push_back( data     &0xFF);
            data_.push_back((data>> 8)&0xFF);
            data_.push_back((data>>16)&0xFF);
            
            ++state_;
            //            std::cout << "D *** msg #" << state_ << std::endl;;
            if (state_ == n_data_words_) {
              std::cout << "D *** decode" << std::endl;;
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
          std::cout << boost::format("%3d") % hist[i];
        const int idx_max = std::distance(hist, std::max_element(hist, hist+30));
        std::cout << " | " << idx_max << " " << hist[idx_max] << std::endl;
        if (hist[idx_max] > 5) {
          sync_offset_ = idx_max;
          state_ = (state_ == -4 ? -3 : state_);
        } else {
          state_ = -4;
        }
      }      
    }

    struct __attribute__((__packed__)) msg_1  {
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

      size_t prc1_     : 16;
      size_t sat_id1_  :  5;
      size_t udre1_    :  2;
      size_t sf1_      :  1;

      size_t sat_id2_  :  5;
      size_t udre2_    :  2;
      size_t sf2_      :  1;
      size_t iod1_     :  8;
      size_t rrc1_     :  8;

      size_t rrc2_     :  8;
      size_t prc2_     : 16; 

      size_t prc3_up_  :  8;
      size_t sat_id3_  :  5;
      size_t udre3_    :  2;
      size_t sf3_      :  1;
      size_t iod2_     :  8;

      size_t iod3_     :  8;
      size_t rrc3_     :  8;
      size_t prc3_low_ :  8;

      static boost::int32_t conv(boost::uint64_t x, int bits) {
        const boost::uint64_t pow2 = (1<<(bits-1));
        return ((x & pow2)
                ? (-1 - ((x-pow2) ^ (pow2-1)))
                : x);
      }

    } ;
    struct __attribute__((__packed__)) msg_7 {
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
      size_t lon_h_     :  8;
      size_t lat_       : 16;

      size_t freq_h_    :  6;
      size_t range_     : 10;
      size_t lon_l_     :  8;

      size_t  bc_coding_ :  1;
      size_t  sync_type_ :  1;
      size_t  mod_code_  :  1;
      size_t  bitrate_   :  3;

      size_t bcid_      : 10;
      size_t health_    :  2;
      size_t freq_l_    :  6;
    } ;

    void decode_message() const {
      switch (msg_type_) {
      case 7: {
        for (size_t i=0; i<data_.size(); i+=9) {
          const msg_7 *m = (const msg_7*)(&data_[i]);
          std::cout << "D*** MSG_7:" 
                    << m->lat() << " "
                    << m->lon() << " "
                    << m->range() << " "
                    << m->freq()  << " "
                    << m->bcid() << " | ";
          for (int j=0; j<9; ++j)
            std::cout << int(data_[i+j]) << " ";
          std::cout << std::endl;
        }
        break;
        case 1:
        case 9: {
          for (size_t i=0; i<data_.size(); i+=15) {
            const msg_1 *m = (const msg_1*)(&data_[i]);
            std::cout << "D*** MSG_1:"
                      << boost::format(" G%02d SF=%d UDRE=%2d PRC=%+8.2f RRC=%+6.3f IOD=%3d")
              % m->sat_id1() % m->sf1() % m->udre1() % m->prc1() % m->rrc1() % m->iod1()
                      << std::endl;
            std::cout << "D*** MSG_1:"
                      << boost::format(" G%02d SF=%d UDRE=%2d PRC=%+8.2f RRC=%+6.3f IOD=%3d")
              % m->sat_id2() % m->sf2() % m->udre2() % m->prc2() % m->rrc2() % m->iod2()
                      << std::endl;
            std::cout << "D*** MSG_1:"
                      << boost::format(" G%02d SF=%d UDRE=%2d PRC=%+8.2f RRC=%+6.3f IOD=%3d")
              % m->sat_id3() % m->sf3() % m->udre3() % m->prc3() % m->rrc3() % m->iod3()
                      << std::endl;
          }
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
    int   sync_counter_;
    int   sync_offset_;

    int   n_data_words_;
    frame frame_;
    boost::uint8_t  msg_type_;
    boost::uint16_t stn_num_;
    boost::uint32_t z_count_;
    boost::int16_t   seq_;
    boost::uint8_t  num_;
    std::vector<boost::uint8_t> data_;
  } ;
} // namespace decode

#endif // _DECODE_RTCM2_HPP_cm160405_
