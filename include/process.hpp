// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
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
#ifndef _PROCESS_HPP_cm160428_
#define _PROCESS_HPP_cm160428_

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

class process : private boost::noncopyable {
public:
  typedef boost::shared_ptr<process> sptr;

  process() {}
  virtual ~process() {}

  virtual bool is_running() const { return true; }
  
protected:
private:
} ;

#endif // _PROCESS_HPP_cm160428_
