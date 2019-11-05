1
// https://stackoverflow.com/questions/57829000/how-to-bind-two-nested-structures-using-pybind11

welcome to SO!

Hum, it should work. My guess is that your python code doesn't instantiate the class values correctly.

I have

#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py=pybind11;

typedef struct _Names
{
    int a;
    int b;
    int c;
} Names;

typedef struct _values
{
    Names names;
} values;


// Wrapping code
PYBIND11_MODULE(wrapper, m) {
    py::class_<Names>(m, "Names")
        .def(py::init<>())
        .def_readwrite("a", &Names::a)
        .def_readwrite("b", &Names::b)
        .def_readwrite("c", &Names::c);

    py::class_<values>(m, "values")
        .def(py::init<>())
        .def_readwrite("names", &values::names);
}

And if I compile it (see example 1 in this github repo if you need directions) to a file called wrapper, I can use it from python as

import wrapper

b = wrapper.values()  # <-- don't forget the parenthesis !
b.names.a = 30
print(b.names.a)  # prints 30

I'm guessing you didn't put parenthesis after wrapper.values which causes (in my case) b to contain the class definition instead of an instance of it.

Hope it helps :)

