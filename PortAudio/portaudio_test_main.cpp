#include <iostream>

#include "portaudio.hpp"

typedef struct iq {
  float i;
  float q;
} iq;

int main()
{
  try {
    portaudio::init::sptr pa(portaudio::init::make());
    const portaudio::init::device_list dl(pa->get_device_list("UDA1361 Eingang"));
    portaudio::device_info::sptr di;
    for (portaudio::init::device_list::const_iterator i(dl.begin());
	 i != dl.end(); ++i) {
      di = *i;
      std::cout << "i= " << (*i)->to_string() << std::endl;
    }
    std::cout << pa->version_number() << " "
	      << pa->version_text() << std::endl;

    const double sample_rate(96e3);
    const unsigned frames(sample_rate/100);

    portaudio::stream_parameters::sptr 
      input_parameters(portaudio::stream_parameters::make(di->index(),
							  2,
							  paFloat32,
							  di->default_high_input_latency()));
    portaudio::stream_parameters::sptr output_parameters;
    std::cout << (pa->is_format_supported(input_parameters,
					  output_parameters,
					  sample_rate) ? "supp" : "not supp") << std::endl;
    
    {
      iq buffer[frames];      
      portaudio::stream_blocking::sptr
	sb(portaudio::stream_blocking::make(input_parameters,
					    output_parameters,
					    sample_rate,
					    frames,
					    paNoFlag));
      sb->start();    
      try {
	sb->read_data(&buffer[0], frames);
      } catch (const portaudio::input_overflow& err) {
	std::cout << err.what() << std::endl;
      }
      sb->stop();
      // for (unsigned i(0); i<frames; ++i)
      // 	std::cout << buffer[i].i << " " 
      // 		  << buffer[i].q << std::endl;
    }
    {
      portaudio::stream_callback::sptr
	sb(portaudio::stream_callback::make(input_parameters,
					    output_parameters,
					    sample_rate,
					    frames,
					    paNoFlag));
      sb->start();
      pa->sleep(10);
      sb->stop();
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 0;
  }
  return 1;
}
