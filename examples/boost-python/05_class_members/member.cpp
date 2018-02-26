
#include <string>
#include <boost/python.hpp>

class SomeClass{
    public:
        SomeClass(std::string name) : name(name), value(0.0) {}
        std::string const name; // this is readonly and doesn't chagne

        double getNumber() const { return value; }

        void setNumber(double n) {
            if (n > 3.14159){
                n = -1;
            }
            value = n;
        }
        double value; // this can be modified
};

BOOST_PYTHON_MODULE(member){
    /* using namespace boost::python; */
    boost::python::class_<SomeClass>("SomeClass", boost::python::init<std::string>())
        .def_readonly("name", &SomeClass::name)
        .def_readwrite("value", &SomeClass::value);
}
