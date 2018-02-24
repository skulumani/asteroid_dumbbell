#include <boost/python.hpp>

// A function
char const* greet()
{
    return "hello world!";
}

// Boost wrapping to expose for Python
BOOST_PYTHON_MODULE(hello)
{
    using namespace boost::python;
    def("greet", greet);
}
