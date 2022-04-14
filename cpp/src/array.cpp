#include <array>
#include <iostream>
 
int main() {
  std::cout << std::boolalpha;

  std::array<int, 4> numbers {3, 1, 4, 1};
  std::cout << "numbers.empty(): " << numbers.empty() << "  " << numbers.size() << "  " << numbers.max_size() <<'\n';
  for(auto v : numbers) {
    std::cout << v << std::endl;
  }

  std::array<int, 0> no_numbers;
  std::cout << "no_numbers.empty(): " << no_numbers.empty() << "  " << no_numbers.size() << "  " << no_numbers.max_size() << '\n';
  for(auto v : no_numbers) {
    std::cout << v << std::endl;
  }

  std::array<int, 10> empty_numbers;
  std::cout << "empty_numbers.empty(): " << empty_numbers.empty() << "  " << empty_numbers.size() << "  " << empty_numbers.max_size() << '\n';
  for(auto v : empty_numbers) {
    std::cout << v << std::endl;
  }
}
