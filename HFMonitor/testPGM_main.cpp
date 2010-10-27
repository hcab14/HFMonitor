// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$

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
