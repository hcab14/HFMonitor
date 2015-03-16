#include <iostream>

#include "util.hpp"
#include "libusb1.hpp"

int main()
{
  try {
    std::vector<usb_device_handle::sptr> dl(usb_device_handle::get_device_list(0,0));
    std::cout << "dl.size() = " << dl.size() << std::endl;
    for (size_t i=0; i<dl.size(); ++i) {
      std::cout << i << " "
		<< dl[i]->get_serial()<< " "
		<< std::hex << dl[i]->get_vendor_id()<< " "
		<< std::hex << dl[i]->get_product_id()<< " " << std::endl;
      // usb_control::sptr ch(usb_control::make(dl[i]));
    }
  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
  }
}
