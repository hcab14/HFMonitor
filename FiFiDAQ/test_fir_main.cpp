#include <iostream>
#include <fstream>

#include "FIR.hpp"
#include "overlap_save.hpp"

int main()
{
  typedef double float_type;
  const size_t m( 1024+1);
  FIR::Lowpass<float_type> fir(m);
  fir.design(0.05, 0.02);
#if 1
  const size_t l(1024*8);

  overlap_save<float_type> os(l,m);
  os.update_filter_coeff(fir.coeff());

  overlap_save<float_type>::complex_vector_type buffer(l);

  std::cout << "out = [\n";
  for (size_t i=0; i<2; ++i) {
    for (size_t j=0; j<l; ++j) 
      buffer[j]= overlap_save<float_type>::complex_type(drand48()-0.5, drand48()-0.5);

    const overlap_save<float_type>::complex_vector_type out(os.proc(buffer));
    for (size_t j=0; j<l; ++j) {
      std::cout << std::scientific
		<< buffer[j].real() << " " << buffer[j].imag() << " " 
		<< out[j].real() << " " << out[j].imag() << std::endl;
    }
  }
  std::cout << "];\n";
#endif
}
