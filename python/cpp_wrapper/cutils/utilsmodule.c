// TODO: Make this C++ instead of C

#include <Python.h>

static PyObject* utils_run(PyObject* self, PyObject* args)
{
  const char* cmd = NULL;

  if(!PyArg_ParseTuple(args, "s", &cmd))
  {
    return NULL;
  }

  printf("Cmd: %s\n", cmd);

  Py_RETURN_NONE;
}

static PyMethodDef UtilMethods[] = {
  {"run", utils_run, METH_VARARGS, "Run the supplied command"},
  {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC initutils(void)
{
  (void)Py_InitModule("_run", UtilMethods);
}


