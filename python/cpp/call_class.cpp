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

#include <Python.h>
#include <iostream>

static constexpr int STDOUT = 0;

int main(int argc, char* argv[]) {
  PyObject *pName, *pModule, *pDict, *pClass, *pInstance, *pValue;
  int i, arg[8];

  if(argc < 4) {
    fprintf(stderr, "Usage: call python_filename class_name function_name\n");
    return 1;
  }

  Py_Initialize();
  pName = PyUnicode_FromString(argv[1]);

  // Load the module object
  std::cout << "Attempting to load python module: " << PyUnicode_AsUTF8(pName) << std::endl;
  if((pModule = PyImport_Import(pName)) == nullptr) {
    PyErr_PrintEx(STDOUT);
    return -1;
  }

  pDict = PyModule_GetDict(pModule);

  // Build the name of a callable class
  std::cout << "Attempting to access python class: " << argv[2] << std::endl;
  if((pClass = PyDict_GetItemString(pDict, argv[2])) == nullptr || !PyCallable_Check(pClass)) {
    std::cout << "Unable to access python class: " << argv[2] << std::endl;
    PyErr_PrintEx(STDOUT);
    return -1;
  }

  // Create an instance of the class
  pInstance = PyObject_CallObject(pClass, NULL);

  // Build parameter list
  if(argc > 4) {
    for(i = 0; i < argc - 4; i++) {
      arg[i] = atoi(argv[i + 4]);
    }
    // Call a method of the class with two parameters
    pValue = PyObject_CallMethod(pInstance, argv[3], "(ii)", arg[0], arg[1]);

  } else {
    // Call a method of the class with no parameters
    pValue = PyObject_CallMethod(pInstance, argv[3], NULL);
  }

  if(pValue != NULL) {
    printf("Return of call : %ld\n", PyLong_AsLong(pValue));
    Py_DECREF(pValue);
  } else {
    PyErr_Print();
  }

  // Clean up
  Py_DECREF(pModule);
  Py_DECREF(pName);
  Py_Finalize();

  return 0;
}
