#include <iostream>
#include <string>
#include <limits>

int main()
{
//     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), 'a' );    
//     std::cin.ignore();    
  if (std::cin.peek() == 'q')
    std::cout << "bye\n";
  else
    std::cout << "bye :(\n";

}
