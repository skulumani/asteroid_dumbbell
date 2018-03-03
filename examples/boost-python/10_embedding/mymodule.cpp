#include <boost/python.hpp>

#include "mymodule.hpp"

BOOST_PYTHON_MODULE(mymodule) {
    class_<Base>("Base")
        .def("__str__", &Base::name)
        ;
}
