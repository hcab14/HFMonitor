#include <iostream>

#include "portaudio.hpp"

int main()
{
  try {
    portaudio::init::sptr pa(portaudio::init::make());
    const portaudio::init::device_list dl(pa->get_device_list("Built-in Input"));
    portaudio::device_info::sptr di;
    for (portaudio::init::device_list::const_iterator i(dl.begin());
	 i != dl.end(); ++i) {
      di = *i;
      std::cout << "i= " << (*i)->to_string() << std::endl;
    }
    std::cout << pa->version_number() << " "
	      << pa->version_text() << std::endl;

    const double sample_rate(196e3);

    portaudio::stream_parameters::sptr 
      input_parameters(portaudio::stream_parameters::make(di->index(),
							  2,
							  paFloat32,
							  di->default_low_input_latency()));
    portaudio::stream_parameters::sptr output_parameters;
    std::cout << (pa->is_format_supported(input_parameters,
					  output_parameters,
					  sample_rate) ? "supp" : "not supp") << std::endl;

    portaudio::stream_blocking::sptr
      sb(portaudio::stream_blocking::make(input_parameters,
					  output_parameters,
					  sample_rate,
					  sample_rate/10,
					  paNoFlag));

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 0;
  }
  return 1;
}
