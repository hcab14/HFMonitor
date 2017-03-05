// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
#ifndef _OVERLAP_SAVE_BASE_HPP_cm170305_
#define _OVERLAP_SAVE_BASE_HPP_cm170305_

#include <boost/shared_ptr.hpp>
#include "processor.hpp"

namespace filter {
  namespace fir {

    class overlap_save_base {
    public:
      typedef boost::shared_ptr<overlap_save_base> sptr;

      typedef processor::base_iq::complex_type complex_type;
      typedef processor::base_iq::vector_type complex_vector_type;

      overlap_save_base() {}
      virtual ~overlap_save_base() {}

      virtual size_t l() const = 0;
      virtual size_t p() const = 0;

      virtual complex_vector_type::const_iterator begin(size_t idx) const = 0;
      virtual complex_vector_type::const_iterator end  (size_t idx) const = 0;

      virtual std::pair<size_t, double> add_filter(const std::vector<float>& b,
						   double_t offset, size_t decim) = 0;

      virtual void proc(processor::base_iq::const_iterator i0,
			processor::base_iq::const_iterator i1) = 0;

    protected:
    private:
    } ;

  } // namespace fir
} // namespace filter

#endif // _OVERLAP_SAVE_BASE_HPP_cm170305_
