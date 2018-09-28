#include <dlfcn.h>
#include <stdio.h>
#include <vector>

#include "HelloWorldInterface.h"

int main(int argc, char* argv[]) {
  /** Validate inputs */
  if(argc != 2) {
    printf("Usage: %s [shared_object_to_load]\n", argv[0]);
    return -1;
  }
  printf("Loading shared object: %s - ", argv[1]);

  auto dso = dlopen(argv[1], RTLD_NOW);
  if(dso == nullptr) {
    printf("FAILED (%s)\n", dlerror());
    return -1;
  }
  printf("SUCCESS\n");

  /** Let's get a valid method from the shared object */
  char* error;
  auto methods = {"ctest1", "hello_world", "newInstance"};
  for(auto& method : methods) {
    auto fn = dlsym(dso, method);
    if((error = dlerror()) != NULL) {
      printf("Failed to find '%s' - %s\n", method, error);
      continue;
    }

    printf("Found '%s'", method);
    if(std::string("hello_world").compare(method) == 0) {
      auto hello = reinterpret_cast<int (*)(void)>(fn);
      printf(" - %d", hello());
    } else if(std::string("newInstance").compare(method) == 0) {
      char* error;
      auto newInstance = reinterpret_cast<std::unique_ptr<HelloWorldInterface> (*)(void)>(dlsym(dso, "newInstance"));
      if((error = dlerror()) != NULL) {
        printf(" - FAILED -> %s", error);
      } else {
        printf(" - ");
        for(auto& l : newInstance()->get_list()) {
          printf("%s ", l.c_str());
        }
      }
    }
    printf("\n");
  }

  /** Close the shared object that was opened */
  printf("Closing shared object: %s - ", argv[1]);
  int ret = dlclose(dso);
  if(ret != 0) {
    printf("FAILED %d - (%s)\n", ret, dlerror());
    return -1;
  }
  printf("SUCCESS\n");

  return 0;
}
