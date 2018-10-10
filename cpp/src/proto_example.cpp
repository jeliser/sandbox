#include "Record01.pb.h"
#include <iostream>

int main(void)
{
  auto obj = WITS0::protobuf::Record01();
  std::cout << obj.has_wellid() << std::endl;
  obj.set_wellid("Hello World");
  std::cout << obj.has_wellid() << "  " << obj.wellid() << std::endl;
} 
