#include <iostream>
#include <memory>

#include <pybind11/pybind11.h>

struct Base {
    Base( void ) {}
    Base( int val_in ) : value(val_in) {}

    virtual ~Base( void ) {}
    
    void print( void ) {
        std::cout << "Value: " << value << std::endl;
    }

    // member attributes
    double value;       
};

void print_shared(std::shared_ptr<Base> ptr) {
    ptr->print();
}

/* int main( void ) { */
/*     std::shared_ptr<Base> ptr; */
/*     ptr = std::make_shared<Base>(10); */
    
/*     print_shared(ptr); */
/*     return 0; */
/* } */

PYBIND11_MODULE(shared_pointer, m) {
    pybind11::class_<Base, std::shared_ptr<Base>>(m, "Base")
        .def(pybind11::init<int>())
        .def("print", &Base::print);

    m.def("print_shared", &print_shared);
}
