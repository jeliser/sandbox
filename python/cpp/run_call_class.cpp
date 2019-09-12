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

extern int run_me(int argc, char* argv[]);


int main(int argc, char* argv[]) {
  return run_me(argc, argv);
}
