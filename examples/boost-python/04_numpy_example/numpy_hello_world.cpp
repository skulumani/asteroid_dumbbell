#include <boost/python/numpy.hpp>
#include <iostream>

namespace p = boost::python;
namespace np = boost::python::numpy;

int main(int argc, char **argv)
{
    Py_Initialize();
    np::initialize();

    p::tuple shape = p::make_tuple(3, 3);
    np::dtype dtype = np::dtype::get_builtin<float>();
    np::ndarray a = np::zeros(shape, dtype);

    // empty array
    np::ndarray b = np::empty(shape, dtype);

    // print out the arrays. Convert array to list then extract each  value
    std::cout << "Original array:\n" << p::extract<char const *>(p::str(a)) << std::endl;

    // reshape teh array into a 1D array
    a = a.reshape(p::make_tuple(9));
    // print it out again
    std::cout << "Reshaped array:\n" << p::extract<char const *>(p::str(a)) << std::endl;

    return 0;
}
