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

#include <boost/thread/mutex.hpp>

#include "shared_memory_mutex.hpp"

int main(int argc, char* argv[])
{
  try {
    cuda_mutex::get().unlock();
    boost::interprocess::scoped_lock<cuda_mutex::mutex_type> lock(cuda_mutex::get());
    std::cout << "locked" << std::endl;
  } catch (...) {
    std::cout << "XXX\n";
  }
  return 1;
}
