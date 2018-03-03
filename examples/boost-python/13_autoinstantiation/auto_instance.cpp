#include <boost/python.hpp>
#include <iostream>
#include "myextensino.hpp"

#if PY_MAJOR_VERSION >= 3
#   define INIT_MODULE  PyInit_myextension
    extern "C" PyObject* INIT_MODULE();
#else
#   define INIT_MODULE initmyextension
    extern "C" void INIT_MODULE();
#endif

namespace bp = boost::python;

int main(int argc, char** argv) {
    try  {
        PyImport_AppendInittab((char *)"myextension", INIT_MODULE);
        Py_Initialize();
        bp::object main_module = bp::import("__main__");
    }
}
