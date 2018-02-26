#include <string>
#include <boost/python.hpp>

class SomeClass{
    public:
        SomeClass(std::string name) : name(name), value(0.0), num(3.141519) {}
        std::string name; // this is readonly and doesn't chagne
   
        // these two methods all us to modify the private attribute value
        double getNumber() const { return value; }

        void setNumber(double n) {
            if (n > 3.14159){
                n = -1;
            }
            value = n;
        }

        // a public attribute that is not writable from python
        double num;
    private:
        double value; // this can be modified
};

BOOST_PYTHON_MODULE(member){
    /* using namespace boost::python; */
    boost::python::class_<SomeClass>("SomeClass", boost::python::init<std::string>())
        .def_readwrite("name", &SomeClass::name)
        .def_readwrite("num", &SomeClass::num)
        .add_property("value", &SomeClass::getNumber, &SomeClass::setNumber);
}
