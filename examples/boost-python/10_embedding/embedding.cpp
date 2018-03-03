#include <boost/python.hpp>
#include <iostream>
#include <frameobject.h>
#include "mymodule.hpp"

namespace bp = boost::python;

#if PY_MAJOR_VERSION >= 3
#   define INIT_MODULE PyInit_mymodule
    extern "C" PyObject* INIT_MODULE();
#else
#   define INIT_MODULE initmymodule
    extern "C" void INIT_MODULE();
#endif


int main(int argc, char** argv) {
    try {
        PyImport_AppendInittab((char*)"mymodule", INIT_MODULE);
        Py_Initialize();
        bp::object main_module = bp::import("__main__");
        bp::dict main_namespace = bp::extract<bp::dict>(main_module.attr("__dict__"));
        bp::object mymodule = bp::import("mymodule");

        main_namespace["precreated_object"] = Base("created on C++ side");
        exec_file("embedding.py", main_namespace, main_namespace);
    } catch (bp::error_already_set& e) {
        PyErr_PrintEx(0);
        return 1;
    }
    return 0;
}
