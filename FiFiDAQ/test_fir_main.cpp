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
  typedef overlap_save<float_type> ols;
  std::cerr << "do " << 100 << " " << ols::design_optimal(100) << std::endl;
  std::cerr << "do " << 200 << " " << ols::design_optimal(200) << std::endl;
  std::cerr << "do " << 500 << " " << ols::design_optimal(500) << std::endl;
  std::cerr << "do " << 1000 << " " << ols::design_optimal(1000) << std::endl;
  std::cerr << "do " << 2000 << " " << ols::design_optimal(2000) << std::endl;
  std::cerr << "do " << 5000 << " " << ols::design_optimal(5000) << std::endl;
  std::cerr << "do " << 10000 << " " << ols::design_optimal(10000) << std::endl;
  std::cerr << "do " << 50000 << " " << ols::design_optimal(50000) << std::endl;
}
