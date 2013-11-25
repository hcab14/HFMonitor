// -*- C++ -*-
// $Id$

#include <math.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>
#include <vector>

#include <boost/circular_buffer.hpp>

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

int main()
{
  const size_t N(1000);
  std::vector<int> bits(N);
  for (size_t i(0); i<N; ++i)
    bits[i] = (drand48() > 0.5);

  const size_t period(20);
  std::vector<int> signal(period*N);
  for (size_t i(0),j(0); i<N; ++i)
    for (size_t k(0); k<period; ++k,++j)
      signal[j] = 2*bits[i]-1;

  early_late_synch els(period);

  for (size_t i=0; i<2; ++i)
    els.insert_signal(-1);
  for (size_t i(0),n(signal.size()),j(0); i<n; ++i) {
    els.insert_signal(signal[i]);
    if (els.bit_valid()) {
      std::cout << "bit: " << els.current_bit() << " " << bits[j++] << std::endl;
    }
  }
  
  return 1;
}
