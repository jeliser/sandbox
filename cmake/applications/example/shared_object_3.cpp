#include "shared_object.h"

// This function is forcing all the methods in the Interface to be implemented.
std::unique_ptr<HelloWorldInterface> newInstance() {
  return std::make_unique<HelloWorld>();
}
