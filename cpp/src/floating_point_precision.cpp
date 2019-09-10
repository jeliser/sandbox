#include <iostream>

int main(int argc, char* argv[]) {
  std::cout << "Why data representations matter --- 4200530431.0000" << std::endl;

  // x86_64
  {
    std::cout << "\nFixed point\n";
    long long i1 = 4200530431.0000;
    long long i2 = std::stoull("4200530431.0000");
    std::cout << "CORRECT: " << i1 << "  " << i2 << std::endl;
    printf("CORRECT: %lld  %lld\n", i1, i2);
  }

  {
    std::cout << "\nFloating point\n";
    float f1 = 4200530431.0000;
    float f2 = std::stof("4200530431.0000");
    std::cout << "WRONG:   " << std::fixed << f1 << "  " << f2 << std::endl;
    printf("WRONG:   %f  %f\n", f1, f2);
  }

  try {
    std::cout << "\nFixed point\n";
    long i1 = 4200530431.0000;
    long i2 = std::stoi("4200530431.0000");
    std::cout << "CORRECT: " << i1 << "  " << i2 << std::endl;
    printf("CORRECT: %ld  %ld\n", i1, i2);
  } catch(const std::exception& e) {
    printf("CORRECT: The number should overflow - %s\n", e.what());
  }

  {
    std::cout << "\nFixed point\n";
    long long i1 = 4200530430.0000;
    long long i2 = std::stoull("4200530430.0000");
    std::cout << "CORRECT: " << i1 << "  " << i2 << std::endl;
    printf("CORRECT: %lld  %lld\n", i1, i2);
  }

  // x86
  {
    std::cout << "\nFixed point\n";
    long i1 = 4200530431.0000;
    long i2 = std::stol("4200530431.0000");
    std::cout << "CORRECT: " << i1 << "  " << i2 << std::endl;
    printf("CORRECT: %ld  %ld\n", i1, i2);
  }

  {
    std::cout << "\nFloating point\n";
    float f1 = 4200530431.0000;
    float f2 = std::stof("4200530431.0000");
    std::cout << "WRONG:   " << std::fixed << f1 << "  " << f2 << std::endl;
    printf("WRONG:   %f  %f\n", f1, f2);
  }

  try {
    std::cout << "\nFixed point\n";
    long i1 = 4200530431.0000;
    long i2 = std::stoi("4200530431.0000");
    std::cout << "CORRECT: " << i1 << "  " << i2 << std::endl;
    printf("CORRECT: %ld  %ld\n", i1, i2);
  } catch(const std::exception& e) {
    printf("CORRECT: The number should overflow - %s\n", e.what());
  }

  {
    std::cout << "\nFixed point\n";
    long i1 = 4200530430.0000;
    long i2 = std::stol("4200530430.0000");
    std::cout << "CORRECT: " << i1 << "  " << i2 << std::endl;
    printf("CORRECT: %ld  %ld\n", i1, i2);
  }

  return 0;
}
