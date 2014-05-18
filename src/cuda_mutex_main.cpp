// -*- C++ -*-
// $Id$
//
// Copyright 2014 Christoph Mayer
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

#include <pthread.h>
#include <signal.h>

#include <iostream>
#include <string>

#include "shared_memory_mutex.hpp"

int main(int argc, char* argv[])
{
  try {
    cuda_mutex::get();
    std::cout << "cuda_mutex already exists" << std::endl;
  } catch (const std::exception& e) {
    sigset_t wait_mask;
    sigaddset(&wait_mask, SIGINT);
    sigaddset(&wait_mask, SIGQUIT);
    sigaddset(&wait_mask, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &wait_mask, 0);

    shm_manager_parent s(cuda_mutex::name());
    
    std::cout << "new cuda_mutex create" << std::endl;

    int sig(0);
    sigwait(&wait_mask, &sig);
  }
  return 1;
}
