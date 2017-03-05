// -*- C++ -*-
// $Id$

#ifndef _ALIGNED_VECTOR_cm150813_
#define _ALIGNED_VECTOR_cm150813_

#include <vector>
#include <boost/align/aligned_allocator.hpp>

template<class T, std::size_t Alignment = 32>
using aligned_vector = std::vector<T, boost::alignment::aligned_allocator<T, Alignment> >;

#endif // _ALIGNED_VECTOR_cm150813_
