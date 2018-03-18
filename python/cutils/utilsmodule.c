// TODO: Make this C++ instead of C

#include <Python.h>

static PyObject *
run_cmd(PyObject* self, PyObject* args)
{
  const char* cmd = NULL;

  if(!PyArg_ParseTuple(args, "s", &cmd))
  {
    return NULL;
  }

  printf("Cmd: %s\n", cmd);

  Py_RETURN_NONE;
}

static PyMethodDef UtilsMethods[] = {
  {"run_cmd", run_cmd, METH_VARARGS, "Run the supplied command"},
  {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC
initutils(void)
{
  (void)Py_InitModule("run_cmd", UtilsMethods);
}


