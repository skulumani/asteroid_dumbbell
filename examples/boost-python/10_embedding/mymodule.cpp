#include <boost/python.hpp>

#include "mymodule.hpp"

BOOST_PYTHON_MODULE(mymodule) {
    boost::python::class_<Base>("Base")
        .def("__str__", &Base::name)
        ;
}
