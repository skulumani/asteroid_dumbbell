#include <boost/python/numpy.hpp>
#include <iostream>

namespace p = boost::python
namespace np = boost::python::numpy

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
    p::object own;


}
