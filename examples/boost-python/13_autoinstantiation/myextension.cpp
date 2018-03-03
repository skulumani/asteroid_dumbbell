#include <boost/python.hpp>
#include "myextension.hpp"

namespace bp = boost::python;

BOOST_PYTHON_MODULE(myextension) {
    bp::class_<Base>("Base", bp::init<std::string>())
        .def("__str__", &Base::name)
        ;
}
