// A sample of python embedding (calling python classes from within C++ code)
//
//  > PYTHONPATH=$( pwd ) ./call_class py_class Multiply multiply
//  Attempting to load python module: py_class
//  The result of 6 x 5 : 30
//  Return of call : 30
//
//  > PYTHONPATH=$( pwd ) ./call_class py_class Multiply multiply2 4 4
//  Attempting to load python module: py_class
//  The result of 4 x 4 : 16
//  Return of call : 16
//
// ------------------------------
//  Off-nominal
// ------------------------------
//
//  > ./call_class py_class Multiply2 multiply2 4 4
//  Attempting to load python module: py_class
//  Original exception was:
//  ModuleNotFoundError: No module named 'py_class'
//

#include <iostream>
#include <dlfcn.h>

int main(int argc, char* argv[]) {

  auto dso = dlopen("./libcall_class.so", RTLD_NOW | RTLD_GLOBAL);
  if(dso == nullptr) {
    std::cout << dlerror() << std::endl;
    return -1;
  }

  char* error;
  std::string method = "run_me";
  dlsym(dso, method.c_str());
  if((error = dlerror()) != NULL) {
    std::cout << "Failed to find '" << method << "' - " << error << std::endl;
    return -1;
  }

  auto callback = reinterpret_cast<int (*)(int, char**)>(dlsym(dso, method.c_str()));
  callback(argc, argv);

  return 0;
}
