#include <iostream>
#include <string>


struct Base {
    virtual std::string name() const { return "Base"; }
    virtual ~Base() {}
};

struct Derived : Base {
    virtual std::string name() const { return "Derived"; }
};

void fb(Base *b) {
    std::cout << (*b).name() << " called." << std::endl;
}

void fd(Derived *d) {
    std::cout << "Derived " << d->name() << " called." << std::endl;
}


Base* factory() {
    return new Derived;
}

#include <boost/python.hpp>
namespace bp = boost::python;

BOOST_PYTHON_MODULE(inheritance) {
    bp::class_<Base, boost::noncopyable>("Base")
        .def("name", &Base::name);
    
    bp::class_<Derived, bp::bases<Base> >("Derived")
        ;

    bp::def("fb", fb);
    bp::def("fd", fd);
    // returning a new instance of Derived. Python will adopt a pointer to Base
    // and hole the new instance in a Python Base object until it is destroyed
    // in python
    bp::def("factory", factory, bp::return_value_policy<bp::manage_new_object>());
}
