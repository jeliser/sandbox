#include "shared_object.h"

int hello_world() {
  return 12345;
}

void HelloWorld::print() {
  printf("How are you?");
}

std::list<std::string> HelloWorld::get_list() {
  return std::list<std::string>({"hello", "world", "this", "is", "a", "list"});
}

// This function is forcing all the methods in the Interface to be implemented.
std::unique_ptr<HelloWorldInterface> newInstance() {
  return std::make_unique<HelloWorld>();
}
