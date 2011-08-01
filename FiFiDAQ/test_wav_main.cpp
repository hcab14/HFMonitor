// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
#include <iostream>

#include "wave_reader.hpp"

class test_proc {
public:
  void process_iq(processor::service_base::sptr service,
                  IQBuffer::Samples::const_iterator i0, 
                  IQBuffer::Samples::const_iterator i1) {
    std::cout << "test_proc::process_iq " << service->approx_ptime() << std::endl;
  }
protected:
private:
} ;

int main(int argc, char* argv[])
{
  LOGGER_INIT("./Log", argv[0]);
  test_proc p;
  wave::reader_iq<test_proc> wr(p, 0.0, boost::posix_time::ptime(), 1.0, 0.0);
  wr.process_file("wav/iq_300kHz.wav");
  std::cout << "samples  " << wr.samples().size() << std::endl;

  wave::reader_perseus<test_proc> wrp(p, 1.0, 0.0);
  return 0;
}
