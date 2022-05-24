
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>

struct Functor {
  explicit Functor(uint32_t attr) : attr(attr) {}
  void operator () (uint32_t index) const {
    std::cout << index << "  " << attr << std::endl;
  }
  uint32_t attr;
};

int main(void) {

  Functor fn(1234);

  fn(999);

  exit(EXIT_SUCCESS);
}
