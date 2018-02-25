#include <boost/python/numpy.hpp>
#include <iostream>

namespace p = boost::python;
namespace np = boost::python::numpy;

int main(int argc, char **argv)
{
    Py_Initialize();
    np::initialize();
    // create an numpy nd array from a tuple
    p::object tu = p::make_tuple('a', 'b', 'c');
    np::ndarray example_tuple = np::array(tu);

    // now create an array from a list
    p::list l;
    l.append('a');
    np::ndarray example_list = np::array(l);

    // specify a dtype for the array
    np::dtype dt = np::dtype::get_builtin<int>();
    np::ndarray example_list1 = np::array(l, dt);

    // create an array by supplying data
    int data[] = {1, 2, 3, 4, 5};

    // create shape and strides required by the function
    p::tuple shape = p::make_tuple(5);
    p::tuple stride = p::make_tuple(sizeof(int));

    // shpae is (4,) while the stride is the number of bytes required to travel
    // to the next element while builing the ndarray
    // the ufnction needs an owner
    p::object own;
    // turn all of this into an array 
    np::ndarray data_ex = np::from_data(data, dt, shape, stride, own);

    // print the array
    std::cout << "Single dimensional array ::" << std::endl
        << p::extract<char const *>(p::str(data_ex)) << std::endl;

    // now create a 3x2 array
    uint8_t mul_data[][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {1, 3, 5, 7}};
    shape = p::make_tuple(3, 2);
    stride = p::make_tuple(sizeof(uint8_t)*2, sizeof(uint8_t));

    // get the dtype
    np::dtype dt1 = np::dtype::get_builtin<uint8_t>();

    // create and print out the ndarray
    np::ndarray mul_data_ex = np::from_data(mul_data, dt1,
            p::make_tuple(3, 4),
            p::make_tuple(4, 1),
            p::object());
    std::cout << "Original multi dimensional array :: " << std::endl
        << p::extract<char const *>(p::str(mul_data_ex)) << std::endl;

    mul_data_ex = np::from_data(mul_data, dt1, shape, stride, p::object());
    std::cout << "Selective multidimensional array :: " << std::endl
        << p::extract<char const *>(p::str(mul_data_ex)) << std::endl;
}
