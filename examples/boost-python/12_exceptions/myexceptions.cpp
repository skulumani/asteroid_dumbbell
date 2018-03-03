#include "Python.h"
#include "boost/python.hpp"

struct OutOfSteakException {};

void translateException( const OutOfSteakException& x) {
    PyErr_SetString(PyExc_UserWarning, "The meat is gone, go for the cheese....");
};

void someFunction() {
    throw OutOfSteakException();
};

BOOST_PYTHON_MODULE(myexceptions) {
    boost::python::register_exception_translator<OutOfSteakException>(translateException);

    boost::python::def("someFunction", someFunction);
}
