#include "shared_object.h"

void HelloWorld::print() {
  printf("How are you?");
}

std::list<std::string> HelloWorld::get_list() {
  return std::list<std::string>({"hello", "world", "this", "is", "a", "list"});
}
