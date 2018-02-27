#include <iostream>
#include <string>
#include <boost/python.hpp>
namespace bp = boost::python;

class Base {
    public:
        virtual std::string name() const { return "Base"; }
        virtual ~Base() {}
};

void identify(Base *b);

void identify(Base *b) {
    std::cout << b->name() << " called." << std::endl;
}

// This is a wrapper for the Base class to expose it to Python
struct BaseWrap : Base, bp::wrapper<Base> {
    virtual std::string name() const {
        if (bp::override n = this->get_override("name")) {
            return n();
        }
        return Base::name();
    }

    std::string default_name() const {
        return this->Base::name();
    }
};


// now for the boost wrapping
BOOST_PYTHON_MODULE(virtual) {
    bp::class_<BaseWrap, boost::noncopyable>("Base")
        .def("name", &Base::name, &BaseWrap::default_name)
        ;
    bp::def("identify", identify);
}
