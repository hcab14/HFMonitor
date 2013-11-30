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
#ifndef _DECODE_EFR_HPP_cm131129_
#define _DECODE_EFR_HPP_cm131129_

#include <deque>
#include <algorithm>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

namespace decode {  
  class efr : public boost::noncopyable {
  public:
    typedef boost::shared_ptr<efr> sptr;    
    typedef std::deque<bool>           bit_vector_type;
    typedef std::vector<unsigned char> data_vector_type;
    typedef data_vector_type::const_iterator const_iterator;

    typedef enum {
      STATE_UNLOCKED,
      STATE_LOCKED,
      STATE_FINISH_PACKET
    } state_type;

    state_type get_state() const { return state_; }
    void set_state(state_type s) { state_ = s; }

    bool data_ok() const { return data_ok_; }
    const data_vector_type& data() const { return data_; }
    const_iterator begin() const { return data_.begin(); }
    const_iterator end() const { return data_.end(); }

    static sptr make() {
      return sptr(new efr());
    }

    void push_back(bool bit) {
      bits_.push_back(bit);
      switch (get_state()) {
      case STATE_UNLOCKED:
        data_.clear();
        data_ok_ = false;
        if (bits_.size() == 44) {
          // std::cout << "D ";
          // std::copy(bits_.begin(),    bits_.begin()+11, std::ostream_iterator<bool>(std::cout, " "));
          // std::cout << "\nD ";
          // std::copy(bits_.begin()+33, bits_.begin()+44, std::ostream_iterator<bool>(std::cout, " "));
          // std::cout << "\nS ";
          // std::copy(start_sequence(), start_sequence()+11, std::ostream_iterator<bool>(std::cout, " "));
          // std::cout << std::endl;
          if (std::equal(bits_.begin(),    bits_.begin()+11, start_sequence()) &&
              std::equal(bits_.begin()+11, bits_.begin()+22, bits_.begin()+22) &&
              std::equal(bits_.begin()+33, bits_.begin()+44, start_sequence())) {
            const std::pair<unsigned char, bool> len1(decode_line(bits_.begin()+11));
            const std::pair<unsigned char, bool> len2(decode_line(bits_.begin()+22));
            // std::cout << "D found start: length = "
            //           << int(len1.first) << " "
            //           << int(len2.first) << " "
            //           << len1.second << "," << len2.second
            //           << std::endl;
            if (len1.second && len2.second && len1.first == len2.first) {
              len_ = len1.first;
              for (size_t i(0); i<44; ++i)
                bits_.pop_front();
              set_state(STATE_LOCKED);
              break;
            }
          }
          if (get_state() == STATE_UNLOCKED)
            bits_.pop_front();
        }
        break;
      case STATE_LOCKED:
        if (bits_.size() == 11) {
          const std::pair<unsigned char, bool> b(decode_line(bits_.begin()));
          if (not b.second)
            set_state(STATE_UNLOCKED);
          else {
            if (data_.size() < len_)  {
              data_.push_back(b.first);
            } else {
              // std::cout << "D checksum: "
              //           << (std::accumulate(data_.begin(), data_.end(), 0) % 256) << " == " << b.first << std::endl;
              if (std::accumulate(data_.begin(), data_.end(), 0) % 256 == b.first) {
                // std::cout << "D data= ";
                // std::copy(data_.begin(), data_.end(), std::ostream_iterator<int>(std::cout, " "));
                // std::cout << std::endl;
                set_state(STATE_FINISH_PACKET);
              }
            }
            bits_.clear();
          }
        }
        break;
      case STATE_FINISH_PACKET:
        if (bits_.size() == 11) {
          if (std::equal(bits_.begin(), bits_.begin()+11, finish_sequence())) {
            // std::cout << "D everything OK!\n";
            data_ok_ = true;
          }
          set_state(STATE_UNLOCKED);
          bits_.clear();
        }
        break;
      default:
        ;
      }
    }

  protected:
    std::pair<unsigned char, bool> decode_line(bit_vector_type::const_iterator i) {
      // i[0]==0 and parity check
      // std::cout << "decode_line: ";
      // std::copy(i, i+11, std::ostream_iterator<bool>(std::cout, ""));
      // std::cout << "\ndecode_line: " << (*i) << " " << std::accumulate(i+1, i+9, 0) << " " << *(i+9) << std::endl;
      if ((*i != 0) ||
          (std::accumulate(i+1, i+9, 0) % 2) != *(i+9))
        return std::make_pair(0,false);
      // bits -> 8 bit number
      unsigned char data(0);
      unsigned char p(1);
      for (bit_vector_type::const_iterator j(i+1); j<i+9; ++j) {
        data |= (*j * p);
        p <<= 1;        
      }
      return std::make_pair(data, true);
    }

  private: 
    efr()
      : state_(STATE_UNLOCKED)
      , data_ok_(false) {}

    static bool* start_sequence() {
      static bool start_bits[11]  = { 0, 0,0,0,1,0,1,1,0, 1,1 };
      return start_bits;
    }
    static bool* finish_sequence() {
      static bool finish_bits[11] = { 0, 0,1,1,0,1,0,0,0, 1,1 };
      return finish_bits;
    }

    state_type       state_;
    bit_vector_type  bits_;
    bit_vector_type  start_sequence_;
    size_t           len_; // length of packet;
    data_vector_type data_; // extracted bytes
    bool             data_ok_;
  } ;

} // namespace decode
#endif // _DECODE_EFR_HPP_cm131129_
