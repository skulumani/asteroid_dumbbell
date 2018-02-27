#include <sstream>
#include <string>

class Example {
    Example(std::string n) : mName(n) {}
    std::string mName;

    public:
        std::string name() const { return mName; }
        static Example* factory() {
            return new Example("factory");
        }
        static Example* singleton() {
            static Example instance = Example("singleton");
            return &instance;
        }
};

#include <boost/python.hpp>
namespace bp = boost::python;

BOOST_PYTHON_MODULE(policies) {
    bp::class_<Example>("Example", bp::no_init)
        .def("__str__", &Example::name)
        .def("factory", &Example::factory,
                bp::return_value_policy<bp::manage_new_object>())
        .staticmethod("factory")
        .def("singleton", &Example::singleton,
                bp::return_value_policy<bp::reference_existing_object>())
        .staticmethod("singleton")
        ;
}
