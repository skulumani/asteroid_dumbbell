#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

#include <iostream>

namespace bp = boost::python;
namespace np = boost::python::numpy;

np::ndarray make_zeros(int rows, int cols)
{
    Py_Initialize();
    return np::zeros(bp::make_tuple(rows, cols), np::dtype::get_builtin<float>());
}

void make_ndarray()
{
    Py_Initialize();
    bp::tuple shape = bp::make_tuple(3, 3);
    np::dtype dtype = np::dtype::get_builtin<float>();
    np::ndarray a = np::zeros(shape, dtype);

    // empty array
    np::ndarray b = np::empty(shape, dtype);

    // print out the arrays. Convert array to list then extract each  value
    std::cout << "Original array:\n" << bp::extract<char const *>(bp::str(a)) << std::endl;

    // reshape teh array into a 1D array
    a = a.reshape(bp::make_tuple(9));
    // print it out again
    std::cout << "Reshaped array:\n" << bp::extract<char const *>(bp::str(a)) << std::endl;
}

void create_array()
{
    Py_Initialize();
    // create an numpy nd array from a tuple
    bp::object tu = bp::make_tuple('a', 'b', 'c');
    np::ndarray example_tuple = np::array(tu);

    // now create an array from a list
    bp::list l;
    l.append('a');
    np::ndarray example_list = np::array(l);

    // specify a dtype for the array
    np::dtype dt = np::dtype::get_builtin<int>();
    np::ndarray example_list1 = np::array(l, dt);

    // create an array by supplying data
    int data[] = {1, 2, 3, 4, 5};

    // create shape and strides required by the function
    bp::tuple shape = bp::make_tuple(5);
    bp::tuple stride = bp::make_tuple(sizeof(int));

    // shpae is (4,) while the stride is the number of bytes required to travel
    // to the next element while builing the ndarray
    // the ufnction needs an owner
    bp::object own;
    // turn all of this into an array 
    np::ndarray data_ex = np::from_data(data, dt, shape, stride, own);

    // print the array
    std::cout << "Single dimensional array ::" << std::endl
        << bp::extract<char const *>(bp::str(data_ex)) << std::endl;

    // now create a 3x2 array
    uint8_t mul_data[][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {1, 3, 5, 7}};
    shape = bp::make_tuple(3, 2);
    stride = bp::make_tuple(sizeof(uint8_t)*2, sizeof(uint8_t));

    // get the dtype
    np::dtype dt1 = np::dtype::get_builtin<uint8_t>();

    // create and print out the ndarray
    np::ndarray mul_data_ex = np::from_data(mul_data, dt1,
            bp::make_tuple(3, 4),
            bp::make_tuple(4, 1),
            bp::object());
    std::cout << "Original multi dimensional array :: " << std::endl
        << bp::extract<char const *>(bp::str(mul_data_ex)) << std::endl;

    mul_data_ex = np::from_data(mul_data, dt1, shape, stride, bp::object());
    std::cout << "Selective multidimensional array :: " << std::endl
        << bp::extract<char const *>(bp::str(mul_data_ex)) << std::endl;
}

BOOST_PYTHON_MODULE(test_numpy)
{
    np::initialize();
    bp::def("make_zeros", make_zeros);
}
