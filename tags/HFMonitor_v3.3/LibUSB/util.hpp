// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#ifndef _util_hpp_cm110222_
#define _util_hpp_cm110222_

#include <stdexcept>
#include <boost/current_function.hpp>

#define THROW_SITE_INFO(what)                                           \
  std::string(std::string(what) + "\n" +                                \
              "  in " + std::string(BOOST_CURRENT_FUNCTION) + "\n" +    \
              "  at " + std::string(__FILE__) + ":" + BOOST_STRINGIZE(__LINE__) + "\n")

#define ASSERT_THROW(_x)                                                \
  if (not (_x))                                                         \
    throw std::runtime_error(THROW_SITE_INFO("assertion failed: " + std::string(#_x))); \
  else void(0)

#endif //_util_hpp_cm110222_
