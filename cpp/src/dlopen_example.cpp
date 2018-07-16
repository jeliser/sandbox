#include <dlfcn.h>
#include <stdio.h>
#include <vector>

#include "dso_library_file.h"

int main(int argc, char * argv[]) {

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
  auto methods = { "ctest1", "hello_world" };
  for(auto& method : methods) {
    auto fn = dlsym(dso, method);
    if((error = dlerror()) != NULL) { 
       printf("Failed to find '%s' - %s\n", method, error);
    } else {
       auto hello = reinterpret_cast<int(*)(void)>(fn);
       printf("Found '%s' - %d\n", method, hello());
    }
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
