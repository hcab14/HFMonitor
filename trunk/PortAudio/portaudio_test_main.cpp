#include <iostream>

#include "portaudio.hpp"

int main()
{
  try {
    portaudio::init::sptr pa(portaudio::init::make());
    const portaudio::init::device_list dl(pa->get_device_list("Built-in Input"));
    for (portaudio::init::device_list::const_iterator i(dl.begin());
	 i != dl.end(); ++i)
      std::cout << "i= " << (*i)->to_string() << std::endl;
    std::cout << pa->version_number() << " "
	      << pa->version_text() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 0;
  }
  return 1;
}
