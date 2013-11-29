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
#ifndef _EARLY_LATE_SYNCH_HPP_cm131129_
#define _EARLY_LATE_SYNCH_HPP_cm131129_

#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>
#include <vector>

#include <boost/circular_buffer.hpp>
namespace demod {

  class early_late_synch {
  public:
    typedef boost::circular_buffer<int> hist_type;

    early_late_synch(double period)
    : alpha_(0.5)
    , period_(period)
    , two_periods_(2*lround(period))
    , counter_(0)
    , t_(two_periods_)
    , err_(0)
    , history_(two_periods_)
    , bit_valid_(false)
    , current_bit_(false) {}

    double period() const { return period_; }

    bool bit_valid() const { return bit_valid_; }
    bool current_bit() const { return current_bit_; }

    void insert_signal(double s) {
      if (counter_ == floor(t_)) {
        bit_valid_ = true;
        current_bit_ = history_[two_periods_/4] > 0;
        // history_[floor(t_-two_periods_)] > 0  -> bit
        //  ==> history_.front
        const double early(std::accumulate(history_.begin(), history_.begin()+two_periods_/2, 0.));
        const double late (std::accumulate(history_.begin()+two_periods_/2, history_.end(),   0.));
        err_     = (1-alpha_)*err_ - alpha_*(std::abs(early)-std::abs(late));
        std::cout << "t_,err= " << t_ << " " << err_ << std::endl;
        t_      += period_ - counter_ + 0.5*err_;
        counter_ = 0;
      } else
        bit_valid_ = false;

      history_.push_back(2*(s>0)-1);
      ++counter_;
    }

  protected:

  private:
    double    alpha_;
    double    period_;
    size_t    two_periods_;
    size_t    counter_;
    double    t_;
    double    err_;
    hist_type history_;  
    bool      bit_valid_;
    bool      current_bit_;
  } ;

} // namespace demod
#endif // _EARLY_LATE_SYNCH_HPP_cm131129_
