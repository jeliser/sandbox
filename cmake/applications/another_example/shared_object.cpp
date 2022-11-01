#include "shared_object.h"

#include <experimental/filesystem>

int hello_world() {
  return 12345;
}

void HelloWorld::print() {
  // Check that the build system is actually using c++17
  auto exists = std::experimental::filesystem::exists("/some/made/up/directory");

  printf("How are you?  %s", exists ? "true" : "false");
}

std::list<std::string> HelloWorld::get_list() {
  return std::list<std::string>({"hello", "world", "this", "is", "a", "list"});
}

// This function is forcing all the methods in the Interface to be implemented.
std::unique_ptr<HelloWorldInterface>

newInstance() {
  return std::make_unique<HelloWorld>();
}
