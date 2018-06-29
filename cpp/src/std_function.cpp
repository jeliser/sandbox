// https://stackoverflow.com/questions/7713266/how-can-i-change-the-variable-to-which-a-c-reference-refers

#include <string>
#include <iostream>
#include <functional>

int main() {

  auto func1 = std::function<void(bool)>();

  if(func1)
    std::cout << "We're a function" << std::endl;
  else
    std::cout << "No function" << std::endl;
}

