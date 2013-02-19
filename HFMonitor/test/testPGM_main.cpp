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
#include <iostream>
#include <fstream>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

#include "netpbm.hpp"

int main()
{
  using namespace netpbm;
  pgm_writer::string_vector c;
  c.push_back("Hello");
  c.push_back("World!");
  const size_t width(256);
  
  boost::filesystem::path path("test.pgm");
  const bool file_exists(boost::filesystem::exists(path));
  if (not file_exists) {
    boost::filesystem::fstream fs(path, std::ios::out);
    pgm_writer p(width, fs);
    p.write_header();
  }
  boost::filesystem::fstream fs(path, std::ios::in | std::ios::out);
  pgm_writer p(width, fs); 

  p.read_header();
  boost::mt19937 gen;
  const boost::uniform_int<> dist(0, 255);
  boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(gen, dist);
  for (unsigned u(0); u<100; ++u) {
    std::string line;
    for (unsigned v(0); v<width; ++v)
      line.push_back(die());
    p.write_line(line, u%11==0);
  }
}
