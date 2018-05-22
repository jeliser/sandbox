// g++ asan_test.cpp -g -fsanitize=address -fno-omit-frame-pointer

#include <string>

int main() {

  std::string(100, 't');

  std::string too_long;
  too_long.assign('t', 100);
  // This will crash address sanitizer :(
  //too_long.assign("t", 100);
}

