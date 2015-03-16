// -*- C++ -*-
// $Id$

#include <iostream>
#include <string>
#include "shared_memory_mutex.hpp"


int main(int argc, char* argv[])
{
  if (argc != 2) return 0;

  std::string mode(argv[1]);
  if (mode == "parent") {
    shm_manager_parent s("XX");
    sleep(10);
  }
  if (mode == "child") {
    shm_manager_child s("XX");
  }

  return 1;
}
