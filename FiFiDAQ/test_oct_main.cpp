// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id: test_wav_main.cpp 128 2011-07-20 17:57:47Z cmayer $
//
// Copyright 2010-2011 Christoph Mayer
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
#include <complex>
#include <octave/oct.h>
#include <octave/oct-map.h>
#include <octave/Matrix.h>
#include <octave/Cell.h>
#include <octave/ov.h>
#include <octave/load-save.h>
#include <octave/ls-mat5.h>

int main()
{
    const bool compress(false);
    const bool save_as_floats(false);
    const bool global(false);
  {
#if 1
    octave_value ov(int16NDArray(dim_vector(100,100)));
    std::ofstream ofs("test.mat", std::ios::binary);
    write_header(ofs, load_save_format(LS_MAT5_BINARY));
    save_mat5_binary_element(ofs, ov, "matrix_a", global, true, save_as_floats, compress);
    save_mat5_binary_element(ofs, ov, "matrix_b", global, true, save_as_floats, compress);
    save_mat5_binary_element(ofs, ov, "matrix_c", global, true, save_as_floats, compress);
    
    // Cell c(1,4);
    // c.elem(0) = ov;
    // c.elem(1) = ov;
    // c.elem(2) = ov;
    // c.elem(3) = ov;
    // save_mat5_binary_element(ofs, c, "cell_a", global, false, save_as_floats, compress);
    
    // Octave_map m;
    // m.assign("a", octave_value(1.));
    // m.assign("b", octave_value(2.));
    // m.assign("c", octave_value(4.));
    // m.assign("d", octave_value(8.));

    // save_mat5_binary_element(ofs, m, "struct_a", global, false, save_as_floats, compress);
#endif
  }
  {
#if 0
    std::fstream ifs("test.mat");
    bool swap;
    std::string filename;
    read_mat5_binary_file_header(ifs, swap, true, filename);
    std::cout << "swap " << 1*swap << " "<< filename << std::endl;
    octave_value ov;
    bool global(false);
    std::string name;
    std::streampos pos = ifs.tellg();
    name = read_mat5_binary_element(ifs, "", swap, global, ov);

    Matrix m(ov.matrix_value());
    Matrix mm(m.rows()+1, m.cols(), 1);
    mm.insert(m,0,0);
    ifs.seekp(pos);
    ifs.seekg(pos);
    save_mat5_binary_element(ifs, mm, "matrix_a", global, false, save_as_floats, compress);
    std::cout << "aaa " << m.rows() << "," << m.cols() << std::endl;
    std::cout << "aaa " << mm.rows() << "," << mm.cols() << std::endl;
    while ((name=read_mat5_binary_element(ifs, "", swap, global, ov)) !="")  {
      if (name == "matrix_a") {
        Matrix m(ov.matrix_value());    
        mm.insert(m, 0,0);
        for (int i=0; i<3; ++i) {
          for (int j=0; j<10; ++j) {
            std::cout << i << " "<< j << " "<< mm.elem(i,j) << std::endl;
          }
        }
      }
      std::cout << " |" << name << "|: "; ov.dump(std::cout); std::cout << std::endl;
    }
#endif
  }
}
