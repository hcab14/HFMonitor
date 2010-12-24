// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>
#include "logging.hpp"

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", argv[0]);

  try {
    LOG_INFO("Info");
    LOG_ERROR("Info");
    throw std::runtime_error("Hello");
  } catch (const std::runtime_error& e) {
    LOGGER_ERROR << e.what() << " 2nd arg" << " 3rd arg";
  }
}
