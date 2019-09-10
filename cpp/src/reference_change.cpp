// https://stackoverflow.com/questions/7713266/how-can-i-change-the-variable-to-which-a-c-reference-refers

#include <string>
#include <iostream>
#include <functional>

int main() {
  uint32_t valA = 123;
  uint32_t valB = 987;
  auto ref = std::ref(valA);

  //  123  987  123
  std::cout << valA << "  " << valB << "  " << ref << std::endl;

  //  123  987  987
  ref = std::ref(valB);
  std::cout << valA << "  " << valB << "  " << ref << std::endl;

  //  123  456  456
  ref.get() = 456;
  std::cout << valA << "  " << valB << "  " << ref << std::endl;
}
